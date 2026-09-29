#include "../include/CollectiveTrustService.h"
#include "../include/EvaluationTruth.h"
#include "../include/SensorNoiseService.h"
#include "../../include/AssociationEngines.h"
#include "../../include/DeadReckoningModels.h"
#include <artery/application/CaObject.h>
#include <artery/application/Timer.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/envmod/service/CpObject.h>
#include <artery/traci/VehicleController.h>
#include <artery/utility/IdentityRegistry.h>
#include <omnetpp/csimulation.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

using namespace omnetpp;

namespace artery
{

namespace {

/// One CSV per path, shared by every receiver of the run (the header is written once).
std::ofstream& sharedCsv(const std::string& path, const char* header)
{
    static std::map<std::string, std::ofstream> streams;
    auto it = streams.find(path);
    if (it == streams.end()) {
        it = streams.emplace(path, std::ofstream(path, std::ios::out | std::ios::trunc)).first;
        it->second << header << '\n';
    }
    return it->second;
}

}  // namespace

Define_Module(CollectiveTrustService)

CollectiveTrustService::~CollectiveTrustService() = default;

TrustEngineParams CollectiveTrustService::readParams()
{
    TrustEngineParams p;
    // components (ablation)
    p.enablePeers = par("enablePeers").boolValue();
    p.enablePoseCheck = par("enablePoseCheck").boolValue();
    p.enableAdmissibility = par("enableAdmissibility").boolValue();
    p.enableObjectKinematics = par("enableObjectKinematics").boolValue();
    // Paper 1, unchanged
    p.kinematics.maxPosDev = par("maxPositionDeviation").doubleValue();
    p.kinematics.contextSpeedWeight = par("contextSpeedWeight").doubleValue();
    p.kinematics.contextYawWeight = par("contextYawWeight").doubleValue();
    p.kinematics.speedToleranceMps = par("speedTolerance").doubleValue();
    p.historySize = par("historySize").intValue();
    p.trackTimeout = par("trackTimeout").doubleValue();
    p.defaultVehicleLength = par("defaultVehicleLength").doubleValue();

    // corroboration and states
    p.maxSupport = par("maxSupport").doubleValue();
    p.peerSupport = par("peerSupport").doubleValue();
    p.inclusionPeriod = par("inclusionPeriod").doubleValue();
    p.implausibleBelow = par("implausibleBelow").doubleValue();
    p.corroboratedFrom = par("corroboratedFrom").doubleValue();

    p.maxObjectSpeed = par("maxObjectSpeed").doubleValue();
    p.maxObjectRange = par("maxObjectRange").doubleValue();

    p.maxSyncDt = par("maxSyncDt").doubleValue();
    p.association.kappaGate = par("kappaGate").doubleValue();
    p.association.lambdaV = par("lambdaV").doubleValue();
    p.association.sigmaVel = par("sigmaVel").doubleValue();
    p.association.aMax = par("aMax").doubleValue();

    // The receiver's model of positioning and of CPM object errors is the one the stations are simulated with
    // (SensorNoiseService): one source of truth for the simulated errors and the gates that must tolerate them.
    if (m_noise) {
        p.gnss = SensorNoiseService::gnssModelOf(*m_noise);
        p.radar = SensorNoiseService::radarModelOf(*m_noise);
    }
    p.association.sigmaGPS = p.gnss.positionSigma();
    p.association.sigmaHeading = p.gnss.headingSigma(0.0);
    p.association.headingSpeedDecay = p.gnss.headingSpeedDecay;
    return p;
}

void CollectiveTrustService::initialize()
{
    ItsG5BaseService::initialize();

    if (cModule* noise = getParentModule() ? getParentModule()->getSubmodule("Noise") : nullptr) {
        m_noise = dynamic_cast<SensorNoiseService*>(noise);
    }
    m_engine = std::make_unique<TrustEngine>(readParams(),
                                             DeadReckoningModelFactory::create(par("drAlgorithm").stdstringValue()),
                                             std::make_unique<GreedyAssociation>());
    m_engine->setClaimSink([this](const ClaimResult& r) { onClaim(r); });

    m_cpmEncoding = parseCpmEncoding(par("cpmEncoding").stdstringValue());
    m_defaultLength = par("defaultVehicleLength").doubleValue();
    m_metricsInterval = par("metricsInterval").doubleValue();
    m_metricsWarmup = par("metricsWarmup").doubleValue();
    m_visualize = par("visualizeTrust").boolValue();
    m_claimRecordFile = par("claimRecordFile").stdstringValue();

    m_camSignal = cComponent::registerSignal("CamReceived");
    m_cpmSignal = cComponent::registerSignal("CpmReceived");
    subscribe(m_camSignal);
    subscribe(m_cpmSignal);

    m_controller = getFacilities().get_const_ptr<traci::VehicleController>();
    m_vdp = getFacilities().get_const_ptr<VehicleDataProvider>();
    m_timer = getFacilities().get_const_ptr<Timer>();
    if (cModule* registry = getSimulation()->getSystemModule()->getSubmodule("idRegistry")) {
        m_identities = dynamic_cast<IdentityRegistry*>(registry);
    }
    if (!m_vdp || !m_timer) {
        throw cRuntimeError("CollectiveTrustService requires VehicleDataProvider and Timer facilities");
    }
}

MessageTiming CollectiveTrustService::timing() const
{
    MessageTiming t;
    t.now = simTime().dbl();
    t.nowTaiMs = countTaiMilliseconds(m_timer->getTimeFor(simTime()));
    return t;
}

// ----------------------------------------------------------------------------
// The receiver's own pose, from its GNSS fix (referenced error model)
// ----------------------------------------------------------------------------
const EgoView& CollectiveTrustService::egoView()
{
    const double now = simTime().dbl();
    if (now == m_egoBuiltAt) {
        return m_ego;
    }
    if (!m_grid) {
        m_grid = m_controller ? calibrateGrid(*m_controller->getTraCI(),
                                              traci::position_cast(m_controller->getBoundary(),
                                                                   m_controller->getPosition()))
                              : GridFrame{};
        recordScalar("gridNorthBearingDeg", m_grid->northBearingDeg);
        m_honestReceiver = !truthOf(m_vdp->getStationId()).faulty;
    }
    const GridFrame& grid = *m_grid;

    // The VDP is a snapshot of the last SUMO step: the pose is taken at that time.
    m_ego = EgoView{};
    m_ego.station = m_vdp->getStationId();
    m_ego.time = m_vdp->updated().dbl();
    const GeoPoint front{m_vdp->latitude().value(), m_vdp->longitude().value()};
    m_ego.heading = grid.toTrueHeading(m_vdp->heading().value() * ConstantsWGS84::RAD_TO_DEG);
    m_ego.speed = m_vdp->speed().value();
    const double length = m_controller ? m_controller->getLength().value() : m_defaultLength;
    const GeoPoint trueCentre = Geodesy::offset(front, m_ego.heading, -0.5 * length);
    // The receiver knows its own position only through its GNSS fix (orientation taken as exact, as Artery's CP
    // service does).
    const LocalPoint ownError = m_noise ? m_noise->gnssErrorNow() : LocalPoint{};
    m_ego.centre = Geodesy::fromLocal(trueCentre, ownError);
    m_egoBuiltAt = now;
    return m_ego;
}

void CollectiveTrustService::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (!obj) {
        return;
    }
    if (signal == m_camSignal) {
        handleCam(obj);
    } else if (signal == m_cpmSignal) {
        handleCpm(obj);
    }
}

void CollectiveTrustService::handleCam(cObject* obj)
{
    auto ca = dynamic_cast<CaObject*>(obj);
    if (!ca) {
        return;
    }
    ++m_camsReceived;
    const EgoView& ego = egoView();
    const CamClaim claim = MessageParsers::parseCam(ca->asn1(), timing(), *m_grid, m_defaultLength);
    const GroundTruth& truth = truthOf(claim.author);
    m_camFalsified = truth.camFalsified && claim.state.time >= truth.camFaultStart;
    m_engine->onCam(claim, ego);
}

void CollectiveTrustService::handleCpm(cObject* obj)
{
    auto cp = dynamic_cast<CpObject*>(obj);
    if (!cp) {
        return;
    }
    ++m_cpmsReceived;
    const EgoView& ego = egoView();
    const CpmSnapshot cpm = MessageParsers::parseCpm(cp->asn1(), timing(), *m_grid, m_cpmEncoding);
    if (cpm.author == 0) {
        return;
    }
    m_objectsReceived += cpm.objects.size();
    m_cpmReferenceTai = cpm.referenceTaiMs;
    const CpmOutcome outcome = m_engine->onCpm(cpm, ego);
    recordAssociation(cpm, outcome);
}

void CollectiveTrustService::trigger()
{
}

// ----------------------------------------------------------------------------
// Ground truth and metrics (never visible to the engine)
// ----------------------------------------------------------------------------

/// One attributed confidence: count it against the truth of the message it belongs to.
void CollectiveTrustService::onClaim(const ClaimResult& r)
{
    const double now = simTime().dbl();
    if (m_visualize && r.kind == ClaimKind::Cam) {
        updateVisualization(r.author, r.state);
    }
    // Metrics describe what HONEST receivers conclude: a faulty receiver's own view is itself faulty.
    if (!m_honestReceiver || now < m_metricsWarmup) {
        return;
    }
    const int kind = r.kind == ClaimKind::Cam ? 0 : 1;
    const bool falsified = r.kind == ClaimKind::Cam
                               ? m_camFalsified
                               : EvaluationTruth::isFalsifiedObject(r.author, m_cpmReferenceTai, r.objectId);
    Outcomes& o = m_outcomes[kind][falsified ? 1 : 0];
    ++o.n;
    const bool flagged = r.state == ClaimState::Implausible;
    o.flagged += flagged;
    o.corroborated += r.state == ClaimState::Corroborated;
    o.peerConfirmed += r.peers > 0 && m_engine->params().enablePeers;
    if (flagged) {
        ++o.reason[static_cast<size_t>(r.reason)];
    }

    if (m_claimRecordFile.empty()) {
        return;
    }
    // one row per claim track and metrics interval
    const auto key = std::make_tuple(r.author, kind, r.objectId);
    auto last = m_lastRecorded.find(key);
    if (last != m_lastRecorded.end() && now - last->second < m_metricsInterval) {
        return;
    }
    m_lastRecorded[key] = now;
    sharedCsv(m_claimRecordFile, "time,receiver,author,kind,object_id,ldm_track,falsified,plausibility,corroboration,"
                                 "peers,state,reason")
        << now << ',' << m_ego.station << ',' << r.author << ',' << toString(r.kind) << ',' << r.objectId << ','
        << r.ldmTrack << ',' << falsified << ',' << r.plausibility << ',' << r.corroboration << ',' << r.peers << ','
        << toString(r.state) << ',' << toString(r.reason) << '\n';
}

const CollectiveTrustService::GroundTruth& CollectiveTrustService::truthOf(StationId id)
{
    auto it = m_truth.find(id);
    if (it != m_truth.end()) {
        return it->second;
    }
    GroundTruth truth;
    if (m_identities) {
        auto identity = m_identities->lookup<IdentityRegistry::application>(static_cast<uint32_t>(id));
        if (identity && identity->host) {
            truth.known = true;
            truth.traciId = identity->traci;
            if (cModule* middleware = identity->host->getSubmodule("middleware")) {
                double start = std::numeric_limits<double>::infinity();
                cModule* ca = middleware->getSubmodule("CA");
                if (ca && ca->hasPar("faultMode") && ca->par("faultMode").stdstringValue() != "none") {
                    truth.camFalsified = true;
                    start = std::min(start, ca->par("faultStartTime").doubleValue());
                }
                cModule* noise = middleware->getSubmodule("Noise");
                if (noise && noise->hasPar("gnssFaultMode") && noise->par("gnssFaultMode").stdstringValue() != "none") {
                    truth.camFalsified = true;  // a positioning failure falsifies every CAM (and CPM, via the registry)
                    start = std::min(start, noise->par("gnssFaultStartTime").doubleValue());
                }
                cModule* cp = middleware->getSubmodule("CP");
                const bool cpFaulty = cp && cp->hasPar("cpmFaultMode") && cp->par("cpmFaultMode").stdstringValue() != "none";
                truth.faulty = truth.camFalsified || cpFaulty;
                truth.camFaultStart = truth.camFalsified ? start : 0.0;
            }
        }
    }
    return m_truth.emplace(id, truth).first->second;
}

/// Resolution of anonymous objects against the identity each author recorded before adding errors. Falsified
/// objects have no true identity and are left out.
void CollectiveTrustService::recordAssociation(const CpmSnapshot& cpm, const CpmOutcome& outcome)
{
    if (!m_honestReceiver || simTime().dbl() < m_metricsWarmup) {
        return;
    }
    const double now = simTime().dbl();
    for (size_t j = 0; j < cpm.objects.size() && j < outcome.admissible.size(); ++j) {
        const ObjectClaim& o = cpm.objects[j];
        if (!outcome.admissible[j] || EvaluationTruth::isFalsifiedObject(cpm.author, cpm.referenceTaiMs, o.objectId)) {
            continue;
        }
        const long truth = EvaluationTruth::objectStation(cpm.author, cpm.referenceTaiMs, o.objectId);
        const StationId resolved = outcome.resolvedTo[j];
        if (resolved >= 0) {
            truth == resolved ? ++m_assoc.tp : ++m_assoc.fp;
        } else {
            // resolvable = the true vehicle is the receiver, or a station whose honest CAM track the receiver holds
            bool resolvable = false;
            if (truth >= 0) {
                const GroundTruth& t = truthOf(truth);
                const bool camLies = t.camFalsified && now >= t.camFaultStart;
                resolvable = (truth == m_ego.station) || (m_engine->hasTrack(truth) && !camLies);
            }
            resolvable ? ++m_assoc.fn : ++m_assoc.tn;
        }
    }
}

void CollectiveTrustService::updateVisualization(StationId id, ClaimState state)
{
    const GroundTruth& truth = truthOf(id);
    if (!m_controller || truth.traciId.empty()) {
        return;
    }
    try {
        auto api = m_controller->getTraCI();
        const libsumo::TraCIColor color = state == ClaimState::Implausible    ? libsumo::TraCIColor(255, 0, 0, 255)
                                          : state == ClaimState::Corroborated ? libsumo::TraCIColor(0, 255, 0, 255)
                                                                              : libsumo::TraCIColor(255, 255, 0, 255);
        api->vehicle.setColor(truth.traciId, color);
    } catch (...) {
    }
}

void CollectiveTrustService::finish()
{
    ItsG5BaseService::finish();
    const char* kinds[] = {"cam", "object"};
    const char* labels[] = {"honest", "falsified"};
    for (int k = 0; k < 2; ++k) {
        for (int f = 0; f < 2; ++f) {
            const Outcomes& o = m_outcomes[k][f];
            const std::string prefix = std::string("cl_") + kinds[k] + "_" + labels[f] + "_";
            recordScalar((prefix + "n").c_str(), o.n);
            recordScalar((prefix + "flagged").c_str(), o.flagged);
            recordScalar((prefix + "corroborated").c_str(), o.corroborated);
            recordScalar((prefix + "peer_confirmed").c_str(), o.peerConfirmed);
            for (size_t i = 1; i < o.reason.size(); ++i) {
                recordScalar((prefix + "reason_" + toString(static_cast<Reason>(i))).c_str(), o.reason[i]);
            }
        }
    }
    recordScalar("honest_receiver", m_honestReceiver ? 1 : 0);

    // Association (resolution of anonymous objects)
    recordScalar("assoc_tp", m_assoc.tp);
    recordScalar("assoc_fp", m_assoc.fp);
    recordScalar("assoc_fn", m_assoc.fn);
    recordScalar("assoc_tn", m_assoc.tn);

    recordScalar("cams_received", m_camsReceived);
    recordScalar("cpms_received", m_cpmsReceived);
    recordScalar("objects_received", m_objectsReceived);
}

} // namespace artery
