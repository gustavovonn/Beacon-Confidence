#include "../include/FaultyCaService.h"
#include "../include/F2mdFaults.h"
#include "../include/MessageEditing.h"
#include <artery/application/CaObject.h>
#include <artery/application/VehicleDataProvider.h>
#include <cmath>

using namespace omnetpp;

namespace artery
{

Define_Module(FaultyCaService)

void FaultyCaService::initialize()
{
    CaService::initialize();
    m_faultMode = par("faultMode").stdstringValue();
    m_faultStartTime = par("faultStartTime").doubleValue();
    m_faultMagnitude = par("faultMagnitude").doubleValue();
    m_faultBearing = par("faultBearing").doubleValue();
    if (m_faultMode != "none" && !F2md::isSpeedFault(m_faultMode) && !isPositionMode(m_faultMode)) {
        throw cRuntimeError("unknown faultMode '%s'", m_faultMode.c_str());
    }
    if (isPositionMode(m_faultMode) && m_faultBearing < 0.0) {
        m_faultBearing = uniform(0.0, 360.0);  // one offset direction per faulty station
    }
    auto draw = [this](double a, double b) { return uniform(a, b); };
    if (m_faultMode == "const_speed") {
        m_constant = F2md::randomVelocity(draw);
    } else if (m_faultMode == "const_speed_offset") {
        m_constant = F2md::constSpeedOffset(draw);
    }
    m_vdp = getFacilities().get_const_ptr<VehicleDataProvider>();
    m_sigCamSent = cComponent::registerSignal("CamSent");
    cComponent::subscribe(m_sigCamSent, this);
    recordScalar("isFaulty", m_faultMode != "none" ? 1 : 0);
}

void FaultyCaService::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal != m_sigCamSent || simTime() < m_faultStartTime || m_faultMode == "none" || !m_vdp) {
        return;
    }
    auto caObj = dynamic_cast<artery::CaObject*>(obj);
    if (!caObj) {
        return;
    }
    auto& cam = const_cast<vanetza::asn1::Cam&>(caObj->asn1());
    if (isPositionMode(m_faultMode)) {
        falsifyPosition(cam);
        return;
    }
    auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        return;
    }
    // A CAM carries the speed magnitude; the falsified velocity vector is reported through it (the heading is kept,
    // as in F2MD).
    auto draw = [this](double a, double b) { return uniform(a, b); };
    const LocalPoint v = F2md::velocity(m_vdp->speed().value(), m_vdp->heading().value() * ConstantsWGS84::RAD_TO_DEG);
    LocalPoint falsified;
    if (m_faultMode == "const_speed") {
        falsified = m_constant;
    } else if (m_faultMode == "random_speed") {
        falsified = F2md::randomVelocity(draw);
    } else {
        const LocalPoint offset = m_faultMode == "const_speed_offset" ? m_constant : F2md::randomSpeedOffset(draw);
        falsified = LocalPoint{v.east + offset.east, v.north + offset.north};
    }
    auto& speed = hfc.choice.basicVehicleContainerHighFrequency.speed.speedValue;
    speed = std::min<long>(16382, std::lround(Geodesy::norm(falsified) * 100.0));
}

bool FaultyCaService::isPositionMode(const std::string& mode)
{
    return mode == "constant_offset" || mode == "random_noise" || mode == "gaussian_noise" || mode == "gradual_drift";
}

/// Paper 1's Artery position faults (kept for its configurations): metric offsets on the local tangent plane.
void FaultyCaService::falsifyPosition(vanetza::asn1::Cam& cam)
{
    auto& position = cam->cam.camParameters.basicContainer.referencePosition;
    LocalPoint offset;
    if (m_faultMode == "constant_offset") {
        offset = Geodesy::direction(m_faultBearing, m_faultMagnitude);
    } else if (m_faultMode == "random_noise") {
        offset = LocalPoint{uniform(-m_faultMagnitude, m_faultMagnitude), uniform(-m_faultMagnitude, m_faultMagnitude)};
    } else if (m_faultMode == "gaussian_noise") {
        offset = LocalPoint{normal(0.0, m_faultMagnitude), normal(0.0, m_faultMagnitude)};
    } else {  // gradual_drift: reaches faultMagnitude 10 s after the fault starts
        const double elapsed = (simTime() - m_faultStartTime).dbl();
        offset = Geodesy::direction(m_faultBearing, m_faultMagnitude * elapsed / 10.0);
    }
    MessageEditing::shiftReference(position.latitude, position.longitude, offset.east, offset.north);
}

} // namespace artery
