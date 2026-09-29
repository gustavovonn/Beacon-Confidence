#include "../include/SensorNoiseService.h"
#include "../include/EvaluationTruth.h"
#include "../include/F2mdFaults.h"
#include "../include/MessageEditing.h"
#include <artery/application/CaObject.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/envmod/service/CpObject.h>
#include <artery/traci/VehicleController.h>
#include <artery/utility/IdentityRegistry.h>
#include <omnetpp/csimulation.h>
#include <cmath>

using namespace omnetpp;

namespace artery
{

Define_Module(SensorNoiseService)

namespace {

/// The OriginatingVehicleContainer orientation of a CPM, or nullptr.
Vanetza_ITS2_Wgs84AngleValue_t* orientationOf(vanetza::asn1::r2::Cpm& cpm)
{
    auto& containers = cpm->payload.cpmContainers.list;
    for (int i = 0; i < containers.count; ++i) {
        auto* wcc = containers.array[i];
        if (wcc && wcc->containerId == Vanetza_ITS2_CpmContainerId_originatingVehicleContainer &&
            wcc->containerData.present == Vanetza_ITS2_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer) {
            return &wcc->containerData.choice.OriginatingVehicleContainer.orientationAngle.value;
        }
    }
    return nullptr;
}

/// Adds `errorDeg` to a decidegree angle in [0, 3600), leaving sentinels alone.
void addDeciDegrees(long& value, double errorDeg)
{
    if (value < 0 || value >= 3600) {
        return;
    }
    long v = value + std::lround(errorDeg * 10.0);
    v %= 3600;
    value = v < 0 ? v + 3600 : v;
}

}  // namespace

void SensorNoiseService::initialize()
{
    ItsG5BaseService::initialize();
    m_vdp = getFacilities().get_const_ptr<VehicleDataProvider>();
    m_controller = getFacilities().get_const_ptr<traci::VehicleController>();
    m_gem = getFacilities().get_mutable_ptr<GlobalEnvironmentModel>();
    if (cModule* registry = getSimulation()->getSystemModule()->getSubmodule("idRegistry")) {
        m_identities = dynamic_cast<IdentityRegistry*>(registry);
    }

    m_gnssOn = par("gnssErrorModel").stdstringValue() == "veremi";
    m_radarOn = par("radarErrorModel").stdstringValue() == "jacobs";
    m_gnss = gnssModelOf(*this);
    m_radar = radarModelOf(*this);
    m_coverage = coverageOf(*this);

    // VeReMi Extension: one draw per vehicle
    if (m_gnssOn) {
        m_e0 = LocalPoint{uniform(-m_gnss.positionBound, m_gnss.positionBound),
                          uniform(-m_gnss.positionBound, m_gnss.positionBound)};
        m_error = m_e0;
        m_headingE0 = uniform(-m_gnss.headingBoundDeg, m_gnss.headingBoundDeg);
        m_speedE0 = normal(0.0, m_gnss.speedSigma);
    }

    if (hasPar("gnssFaultMode")) {
        m_faultMode = par("gnssFaultMode").stdstringValue();
        m_faultStartTime = par("gnssFaultStartTime").doubleValue();
        if (m_faultMode != "none" && !F2md::isPositionFault(m_faultMode)) {
            throw cRuntimeError("unknown gnssFaultMode '%s'", m_faultMode.c_str());
        }
        if (m_controller) {
            const traci::Boundary b = m_controller->getBoundary();
            m_playgroundWidth = b.upperRightPosition().x - b.lowerLeftPosition().x;
            m_playgroundHeight = b.upperRightPosition().y - b.lowerLeftPosition().y;
        }
        auto draw = [this](double a, double b) { return uniform(a, b); };
        if (m_faultMode == "const_pos") {
            m_faultConstant = LocalPoint{uniform(0.0, m_playgroundWidth), uniform(0.0, m_playgroundHeight)};
        } else if (m_faultMode == "const_pos_offset") {
            m_faultConstant = F2md::constPosOffset(draw);
        }
    }
    recordScalar("isGnssFaulty", m_faultMode != "none" ? 1 : 0);

    m_camSent = cComponent::registerSignal("CamSent");
    m_cpmSent = cComponent::registerSignal("CpmSent");
    subscribe(m_camSent);
    subscribe(m_cpmSent);
}

/// EVALUATION ONLY: the vehicle at playground offset (dx, dy) from this station's reference, before any error.
void SensorNoiseService::recordObjectStation(const vanetza::asn1::r2::Cpm& cpm, uint16_t objectId, double dx, double dy)
{
    if (!m_gem || !m_identities || !m_vdp) {
        return;
    }
    const Position& own = m_vdp->position();
    const double x = own.x.value() + dx;
    const double y = own.y.value() + dy;
    constexpr double r = 1.0;  // [m] the offset is the true centre, rounded to cm
    std::vector<Position> area{Position(x - r, y - r), Position(x + r, y - r), Position(x + r, y + r), Position(x - r, y + r)};
    boost::geometry::correct(area);
    for (const auto& object : m_gem->preselectObjects("", area)) {
        const Position& c = object->getCentrePoint();
        if (std::hypot(c.x.value() - x, c.y.value() - y) <= r) {
            if (auto identity = m_identities->lookup<IdentityRegistry::traci>(object->getExternalId())) {
                EvaluationTruth::setObjectStation(cpm, objectId, static_cast<long>(identity->application));
            }
            return;
        }
    }
}

GnssErrorModel SensorNoiseService::gnssModelOf(cModule& noise)
{
    GnssErrorModel m;
    if (noise.par("gnssErrorModel").stdstringValue() != "veremi") {
        m.positionBound = 0.0;  // an error-free receiver model
        m.jitterFraction = 0.0;
        m.headingBoundDeg = 0.0;
        m.speedSigma = 0.0;
        return m;
    }
    m.positionBound = noise.par("gnssPositionBound").doubleValue();
    m.jitterFraction = noise.par("gnssJitterFraction").doubleValue();
    m.headingBoundDeg = noise.par("gnssHeadingBound").doubleValue();
    m.headingSpeedDecay = noise.par("gnssHeadingSpeedDecay").doubleValue();
    m.speedSigma = noise.par("gnssSpeedSigma").doubleValue();
    return m;
}

RadarErrorModel SensorNoiseService::radarModelOf(cModule& noise)
{
    RadarErrorModel m;
    if (noise.par("radarErrorModel").stdstringValue() != "jacobs") {
        m.rangeSigma = 0.0;
        m.azimuthSigmaDeg = 0.0;
        m.speedSigma = 0.0;
        return m;
    }
    m.rangeSigma = noise.par("radarRangeSigma").doubleValue();
    m.azimuthSigmaDeg = noise.par("radarAzimuthSigma").doubleValue();
    m.speedSigma = noise.par("radarSpeedSigma").doubleValue();
    return m;
}

SensorCoverage SensorNoiseService::coverageOf(cModule& noise)
{
    try {
        return SensorCoverage::parse(noise.par("sensorCoverage").stdstringValue());
    } catch (const std::exception& e) {
        throw cRuntimeError("sensorCoverage: %s", e.what());
    }
}

void SensorNoiseService::finish()
{
    recordScalar("noise_invalid_cpms", m_invalidCpms);
    ItsG5BaseService::finish();
}

/// VeReMi Extension position error, one step per simulation instant: E_t ~ N((E0 + E_{t-1}) / 2, (c E0)^2).
void SensorNoiseService::updateGnssError()
{
    const double now = simTime().dbl();
    if (!m_gnssOn || now == m_errorTime) {
        return;
    }
    auto draw = [this](double mean, double sigma) { return normal(mean, sigma); };
    m_error = LocalPoint{m_gnss.step(m_e0.east, m_error.east, draw), m_gnss.step(m_e0.north, m_error.north, draw)};
    m_errorTime = now;
}

double SensorNoiseService::headingErrorNow() const
{
    return m_gnssOn && m_vdp ? m_gnss.headingError(m_headingE0, m_vdp->speed().value()) : 0.0;
}

/// Position malfunction at the current instant [m east/north]; one sample per instant so CAM and CPM agree.
LocalPoint SensorNoiseService::malfunctionOffset()
{
    const double now = simTime().dbl();
    if (m_faultMode == "none" || now < m_faultStartTime || !m_vdp) {
        return LocalPoint{};
    }
    if (now == m_faultSampleTime) {
        return m_faultSample;
    }
    auto draw = [this](double a, double b) { return uniform(a, b); };
    const Position& own = m_vdp->position();  // playground coordinates (+x east, +y south)
    auto toward = [&own](double x, double y) { return F2md::fromPlayground(x - own.x.value(), y - own.y.value()); };
    if (m_faultMode == "const_pos") {
        m_faultSample = toward(m_faultConstant.east, m_faultConstant.north);
    } else if (m_faultMode == "random_pos") {
        m_faultSample = toward(uniform(0.0, m_playgroundWidth), uniform(0.0, m_playgroundHeight));
    } else if (m_faultMode == "const_pos_offset") {
        m_faultSample = m_faultConstant;
    } else {  // random_pos_offset
        m_faultSample = F2md::randomPosOffset(draw);
    }
    m_faultSampleTime = now;
    return m_faultSample;
}

LocalPoint SensorNoiseService::gnssErrorNow()
{
    Enter_Method_Silent();
    updateGnssError();
    const LocalPoint fault = malfunctionOffset();
    return LocalPoint{(m_gnssOn ? m_error.east : 0.0) + fault.east, (m_gnssOn ? m_error.north : 0.0) + fault.north};
}

void SensorNoiseService::shiftReference(long& latitude, long& longitude)
{
    const LocalPoint e = gnssErrorNow();
    if (e.east != 0.0 || e.north != 0.0) {
        MessageEditing::shiftReference(latitude, longitude, e.east, e.north);
    }
}

GeoPoint SensorNoiseService::radarMeasure(const GeoPoint& hostCentre, double hostHeadingDeg, const GeoPoint& target)
{
    Enter_Method_Silent();
    if (!m_radarOn) {
        return target;
    }
    const LocalPoint local = Geodesy::toLocal(hostCentre, target);
    const LocalPoint measured = m_radar.measure(m_coverage.mountFor(hostHeadingDeg, local), local,
                                                [this](double mean, double sigma) { return normal(mean, sigma); });
    return Geodesy::fromLocal(hostCentre, measured);
}

double SensorNoiseService::radarSpeed(double speed)
{
    Enter_Method_Silent();
    return m_radarOn ? std::max(0.0, speed + normal(0.0, m_radar.speedSigma)) : speed;
}

void SensorNoiseService::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == m_camSent) {
        auto* ca = dynamic_cast<CaObject*>(obj);
        if (!ca) return;
        auto& cam = const_cast<vanetza::asn1::Cam&>(ca->asn1());
        auto& position = cam->cam.camParameters.basicContainer.referencePosition;
        shiftReference(position.latitude, position.longitude);
        auto& hfc = cam->cam.camParameters.highFrequencyContainer;
        if (m_gnssOn && hfc.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
            auto& hf = hfc.choice.basicVehicleContainerHighFrequency;
            addDeciDegrees(hf.heading.headingValue, headingErrorNow());
            if (hf.speed.speedValue != SpeedValue_unavailable) {
                hf.speed.speedValue = std::max(0L, std::lround(hf.speed.speedValue * (1.0 + m_speedE0)));
            }
        }
    } else if (signal == m_cpmSent) {
        auto* cp = dynamic_cast<CpObject*>(obj);
        if (!cp) return;
        auto& cpm = const_cast<vanetza::asn1::r2::Cpm&>(cp->asn1());
        auto& position = cpm->payload.managementContainer.referencePosition;
        shiftReference(position.latitude, position.longitude);  // every object is placed from it
        const LocalPoint fault = malfunctionOffset();
        if (fault.east != 0.0 || fault.north != 0.0) {
            EvaluationTruth::markFalsified(cpm);
        }
        if (auto* orientation = orientationOf(cpm)) {
            addDeciDegrees(*orientation, headingErrorNow());
        }

        auto* poc = MessageEditing::perceivedObjects(cpm);
        if (poc && m_vdp) {
            MessageEditing::canonicalOrder(*poc);  // draws must not depend on heap order
            // CPM offsets are playground deltas (+x east, +y south) from the front bumper; measure every object
            // again from the true sensor mount. Rotation to true north does not matter for distances and angles.
            const GeoPoint front{0.0, 0.0};
            const double heading = m_vdp->heading().value() * ConstantsWGS84::RAD_TO_DEG;
            const double length = m_controller ? m_controller->getLength().value() : 5.0;
            const GeoPoint centre = Geodesy::offset(front, heading, -0.5 * length);
            MessageEditing::filterObjects(*poc, [&](Vanetza_ITS2_PerceivedObject_t& po) {
                if (po.objectId) {
                    recordObjectStation(cpm, static_cast<uint16_t>(*po.objectId), po.position.xCoordinate.value * 0.01,
                                        po.position.yCoordinate.value * 0.01);
                }
                if (!m_radarOn) {
                    return true;
                }
                const GeoPoint target = Geodesy::fromLocal(
                    front, F2md::fromPlayground(po.position.xCoordinate.value * 0.01, po.position.yCoordinate.value * 0.01));
                const LocalPoint d = Geodesy::toLocal(front, radarMeasure(centre, heading, target));
                po.position.xCoordinate.value = MessageEditing::clampCoordinate(std::lround(d.east * 100.0));
                po.position.yCoordinate.value = MessageEditing::clampCoordinate(std::lround(-d.north * 100.0));
                if (po.velocity && po.velocity->present == Vanetza_ITS2_Velocity3dWithConfidence_PR_polarVelocity) {
                    auto& speed = po.velocity->choice.polarVelocity.velocityMagnitude.speedValue;
                    if (speed != Vanetza_ITS2_SpeedValue_unavailable) {
                        speed = std::min<long>(16382, std::lround(radarSpeed(speed * 0.01) * 100.0));
                    }
                }
                return true;
            });
        }
        EvaluationTruth::evictBefore(EvaluationTruth::referenceTime(cpm) - 10000);  // receivers look up fresh CPMs only
        std::string error;
        if (!cpm.validate(error)) {
            ++m_invalidCpms;
            EV_WARN << "CPM invalid after noise injection: " << error << "\n";
        }
    }
}

} // namespace artery
