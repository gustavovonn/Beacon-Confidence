#include "ArteryDatasetGenerator.h"

#include "../../cp-trustfulness/artery/include/EvaluationTruth.h"   // the objectStation oracle
#include "../../cp-trustfulness/include/Geodesy.h"

#include <artery/application/CaObject.h>
#include <artery/envmod/service/CpObject.h>
#include <artery/application/Timer.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/traci/VehicleController.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace artery
{

namespace {

/**
 * One CSV per path, shared by every vehicle in the run, header written once.
 * Pattern lifted from cp-trustfulness/artery/src/CollectiveTrustService.cc. Safe because OMNeT++
 * runs single-threaded: every service instance writes to the same stream in event order.
 */
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

/**
 * Message content is written ONCE; reception is written once per receiver.
 *
 * Every vehicle in range receives the same CAM, so writing content on each reception would multiply
 * the dataset by the neighbourhood size and make every per-message statistic wrong. The content
 * tables are therefore keyed by a synthetic message id and deduplicated here; reception.csv carries
 * the (message, receiver, arrival time) triples, which is the part that genuinely differs.
 */
bool firstSighting(const std::string& messageId)
{
    static std::set<std::string> seen;
    return seen.insert(messageId).second;
}

/**
 * Run-wide totals, describing the dataset rather than any one vehicle.
 *
 * They must be global: message content is deduplicated across receivers, so a per-vehicle counter
 * holds only the messages that vehicle happened to be the first to see. The first version of this
 * file used per-vehicle counters and let "the instance with the highest count" write the manifest;
 * vehicles leave the region throughout the run and call finish() as they go, so the manifest was
 * written by an early departure and reported zeros.
 */
struct Totals {
    uint64_t cams = 0;
    uint64_t cpms = 0;             ///< every CPM recorded, INCLUDING ones carrying no object
    uint64_t cpmsWithObjects = 0;  ///< the subset that produced rows in cpm.csv
    uint64_t objects = 0;
    uint64_t objectsWithOracle = 0;
    uint64_t receptions = 0;
};

Totals& totals()
{
    static Totals t;
    return t;
}

/// kind:author:generation-time-in-ms -- unique per transmitted message, and reproducible.
std::string messageIdOf(const char* kind, long author, double generationTime)
{
    std::ostringstream os;
    os << kind << ':' << author << ':' << static_cast<long long>(std::llround(generationTime * 1000.0));
    return os.str();
}

}  // namespace

Define_Module(ArteryDatasetGenerator)

void ArteryDatasetGenerator::initialize()
{
    ItsG5BaseService::initialize();

    m_vdp = &getFacilities().get_const<VehicleDataProvider>();
    m_timer = &getFacilities().get_const<Timer>();
    m_controller = getFacilities().get_const_ptr<traci::VehicleController>();

    m_outputDir = par("outputDir").stdstringValue();
    m_warmup = par("warmup").doubleValue();
    m_defaultLength = par("defaultVehicleLength").doubleValue();
    m_cpmEncoding = par("cpmEncoding").stdstringValue();

    m_camSignal = cComponent::registerSignal("CamReceived");
    m_cpmSignal = cComponent::registerSignal("CpmReceived");
    subscribe(m_camSignal);
    subscribe(m_cpmSignal);
}

MessageTiming ArteryDatasetGenerator::timing() const
{
    MessageTiming t;
    t.now = omnetpp::simTime().dbl();
    t.nowTaiMs = countTaiMilliseconds(m_timer->getTimeFor(omnetpp::simTime()));
    return t;
}

// to review
const GridFrame& ArteryDatasetGenerator::grid()
{
    if (!m_grid) {
        m_grid = m_controller
                     ? calibrateGrid(*m_controller->getTraCI(),
                                     traci::position_cast(m_controller->getBoundary(),
                                                          m_controller->getPosition()))
                     : GridFrame{};
        recordScalar("gridNorthBearingDeg", m_grid->northBearingDeg);
    }
    return *m_grid;
}

void ArteryDatasetGenerator::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal,
                                           omnetpp::cObject* obj, omnetpp::cObject*)
{
    if (!obj || omnetpp::simTime().dbl() < m_warmup) {
        return;
    }
    if (signal == m_camSignal) {
        handleCam(obj);
    } else if (signal == m_cpmSignal) {
        handleCpm(obj);
    }
}

void ArteryDatasetGenerator::handleCam(omnetpp::cObject* obj)
{
    auto* ca = dynamic_cast<CaObject*>(obj);
    if (!ca) {
        return;
    }
    const CamClaim claim = MessageParsers::parseCam(ca->asn1(), timing(), grid(), m_defaultLength);
    const std::string id = messageIdOf("cam", static_cast<long>(claim.author), claim.state.time);
    const double now = omnetpp::simTime().dbl();

    ++m_receptions;
    ++totals().receptions;
    sharedCsv(m_outputDir + "/reception.csv", "message_id,receiver,t_rx")
        << id << ',' << m_vdp->station_id() << ',' << std::fixed << std::setprecision(4) << now << '\n';

    if (!firstSighting(id)) {
        return;   // another receiver already recorded this message's content
    }
    ++totals().cams;
    sharedCsv(m_outputDir + "/cam.csv",
              "message_id,t,sender,lat,lon,speed,heading,yaw_rate,accel,length")
        << id << ',' << std::fixed << std::setprecision(4) << claim.state.time << ','
        << claim.author << ',' << std::setprecision(9) << claim.state.lat << ','
        << claim.state.lon << ',' << std::setprecision(4) << claim.state.speed << ','
        << claim.state.heading << ',' << std::setprecision(6) << claim.state.yawRate << ','
        << std::setprecision(4) << claim.state.accel << ',' << claim.length << '\n';
}

void ArteryDatasetGenerator::handleCpm(omnetpp::cObject* obj)
{
    auto* cp = dynamic_cast<CpObject*>(obj);
    if (!cp) {
        return;
    }
    const CpmSnapshot cpm =
        MessageParsers::parseCpm(cp->asn1(), timing(), grid(),
                                 parseCpmEncoding(m_cpmEncoding));
    if (cpm.author == 0) {
        return;
    }
    const std::string id = messageIdOf("cpm", static_cast<long>(cpm.author), cpm.time);
    const double now = omnetpp::simTime().dbl();

    ++m_receptions;
    ++totals().receptions;
    sharedCsv(m_outputDir + "/reception.csv", "message_id,receiver,t_rx")
        << id << ',' << m_vdp->station_id() << ',' << std::fixed << std::setprecision(4) << now << '\n';

    if (!firstSighting(id)) {
        return;
    }
    ++totals().cpms;

    if (!cpm.objects.empty()) {
        // Reported separately because the two differ a lot -- roughly half the CPMs on LuST carry
        // no perceived object at all -- and a reader comparing `cpms` against the row count of
        // cpm.csv would otherwise conclude that rows had been lost.
        ++totals().cpmsWithObjects;
    }

    std::ofstream& out = sharedCsv(m_outputDir + "/cpm.csv",
                                   "message_id,t,author,author_lat,author_lon,author_heading,"
                                   "object_id,dx,dy,speed,heading,has_velocity,true_station");
    for (const ObjectClaim& o : cpm.objects) {
        ++totals().objects;

        // Back to the author's frame. parseCpm converts the transmitted offset to an absolute
        // position; toLocal is the exact inverse of the fromLocal it used, around the same origin,
        // so this recovers the offset as sent rather than approximating it.
        const LocalPoint rel = Geodesy::toLocal(cpm.referencePosition, o.position);

        const long truth = EvaluationTruth::objectStation(cpm.author, cpm.referenceTaiMs, o.objectId);
        if (truth != 0) {
            ++totals().objectsWithOracle;
        }

        out << id << ',' << std::fixed << std::setprecision(4) << o.time << ',' << cpm.author << ','
            << std::setprecision(9) << cpm.referencePosition.lat << ',' << cpm.referencePosition.lon
            << ',' << std::setprecision(4) << cpm.heading << ',' << o.objectId << ','
            << rel.east << ',' << rel.north << ',' << o.speed << ',' << o.heading << ','
            << (o.hasVelocity ? 1 : 0) << ',';
        // an explicit empty field, never 0: station 0 is not a valid id and a silent 0 would be
        // read downstream as "this object is station zero" instead of "the oracle did not resolve"
        if (truth != 0) {
            out << truth;
        }
        out << '\n';
    }
}

/**
 * Every frozen parameter, plus the counters. A dataset whose manifest does not match its contents
 * is a dataset that cannot be trusted, and the parser's test checks exactly that.
 */
void ArteryDatasetGenerator::writeMetadata() const
{
    std::ofstream os(m_outputDir + "/metadata.json", std::ios::out | std::ios::trunc);
    if (!os) {
        return;
    }
    os << "{\n";
    os << "  \"generator\": \"ArteryDatasetGenerator\",\n";
    os << "  \"schema\": 1,\n";
    os << "  \"positions\": \"perceived objects are author-relative (dx east, dy north, metres); "
          "author pose is separate\",\n";
    os << "  \"warmup_s\": " << m_warmup << ",\n";
    os << "  \"default_vehicle_length_m\": " << m_defaultLength << ",\n";
    os << "  \"cpm_encoding\": \"" << m_cpmEncoding << "\",\n";
    os << "  \"written_at_sim_time_s\": " << omnetpp::simTime().dbl() << ",\n";
    os << "  \"counts\": {\n";
    os << "    \"cams\": " << totals().cams << ",\n";
    os << "    \"cpms\": " << totals().cpms << ",\n";
    os << "    \"cpms_with_objects\": " << totals().cpmsWithObjects << ",\n";
    os << "    \"objects\": " << totals().objects << ",\n";
    os << "    \"objects_with_oracle\": " << totals().objectsWithOracle << ",\n";
    os << "    \"receptions\": " << totals().receptions << "\n";
    os << "  }\n";
    os << "}\n";
}

void ArteryDatasetGenerator::finish()
{
    // Rewritten on every finish(). Vehicles leave the region throughout the run and each calls
    // finish() as it goes, so there is no single "last" instance to nominate -- but the totals are
    // global and only grow, so the last write is always the complete one.
    writeMetadata();
    recordScalar("receptionsSeen", static_cast<double>(m_receptions));
    ItsG5BaseService::finish();
}

}  // namespace artery
