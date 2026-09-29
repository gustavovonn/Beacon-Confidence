#include "../include/FaultyCpService.h"
#include "../include/EvaluationTruth.h"
#include "../include/F2mdFaults.h"
#include "../include/MessageEditing.h"
#include "../../include/Geodesy.h"
#include <artery/application/VehicleDataProvider.h>
#include <artery/envmod/service/CpObject.h>
#include <vanetza/asn1/cpm.hpp>
#include <cmath>
#include <functional>

using namespace omnetpp;

namespace artery
{

namespace {

/// Playground displacement (+x east, +y south) of `distance` metres along a grid heading.
void gridDisplacement(double gridHeadingDeg, double distance, double& dx, double& dy)
{
    const double h = gridHeadingDeg * ConstantsWGS84::DEG_TO_RAD;
    dx = distance * std::sin(h);
    dy = -distance * std::cos(h);
}

/// Moves every perceived object by a playground offset [m].
void shiftObjects(Vanetza_ITS2_PerceivedObjectContainer_t& poc, const std::function<LocalPoint()>& offset)
{
    MessageEditing::filterObjects(poc, [&offset](Vanetza_ITS2_PerceivedObject_t& po) {
        const LocalPoint o = offset();  // east/north
        po.position.xCoordinate.value = MessageEditing::clampCoordinate(po.position.xCoordinate.value + std::lround(o.east * 100.0));
        po.position.yCoordinate.value = MessageEditing::clampCoordinate(po.position.yCoordinate.value - std::lround(o.north * 100.0));
        return true;
    });
}

}  // namespace

Define_Module(FaultyCpService)

void FaultyCpService::initialize()
{
    CpService::initialize();

    m_cpmFaultMode = par("cpmFaultMode").stdstringValue();
    m_cpmFaultMagnitude = par("cpmFaultMagnitude").doubleValue();
    m_cpmFaultStartTime = par("cpmFaultStartTime").doubleValue();
    if (m_cpmFaultMode != "none" && m_cpmFaultMode != "ghost" && m_cpmFaultMode != "object_const_offset" &&
        m_cpmFaultMode != "object_random_offset") {
        throw cRuntimeError("unknown cpmFaultMode '%s'", m_cpmFaultMode.c_str());
    }
    m_ghostId = static_cast<uint16_t>(intuniform(1, 65535));
    if (m_cpmFaultMode == "object_const_offset") {
        m_constOffset = F2md::constPosOffset([this](double a, double b) { return uniform(a, b); });
    }
    m_vdp = getFacilities().get_const_ptr<VehicleDataProvider>();

    m_sigCpmSent = cComponent::registerSignal("CpmSent");
    cComponent::subscribe(m_sigCpmSent, this);

    recordScalar("isCpFaulty", m_cpmFaultMode != "none" ? 1 : 0);
}

void FaultyCpService::finish()
{
    recordScalar("fault_invalid_cpms", m_invalidCpms);
    CpService::finish();
}

void FaultyCpService::injectGhost(vanetza::asn1::r2::Cpm& cpm, double gridDx, double gridDy, double speed,
                                  double gridHeadingDeg)
{
    auto* poc = MessageEditing::perceivedObjects(cpm, true);
    auto* po = vanetza::asn1::allocate<Vanetza_ITS2_PerceivedObject_t>();
    po->objectId = vanetza::asn1::allocate<Vanetza_ITS2_Identifier2B_t>();
    *po->objectId = m_ghostId;
    po->measurementDeltaTime = 0;
    po->position.xCoordinate.value = MessageEditing::clampCoordinate(std::lround(gridDx * 100.0));
    po->position.xCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;
    po->position.yCoordinate.value = MessageEditing::clampCoordinate(std::lround(gridDy * 100.0));
    po->position.yCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;
    po->objectAge = vanetza::asn1::allocate<Vanetza_ITS2_DeltaTimeMilliSecondSigned_t>();
    *po->objectAge = 1000;
    po->objectPerceptionQuality = vanetza::asn1::allocate<Vanetza_ITS2_ObjectPerceptionQuality_t>();
    *po->objectPerceptionQuality = Vanetza_ITS2_ObjectPerceptionQuality_fullConfidence;
    // A plausible ghost moves like a vehicle; a speed-0 ghost would be trivially separable.
    po->velocity = vanetza::asn1::allocate<Vanetza_ITS2_Velocity3dWithConfidence_t>();
    po->velocity->present = Vanetza_ITS2_Velocity3dWithConfidence_PR_polarVelocity;
    auto& polar = po->velocity->choice.polarVelocity;
    polar.velocityMagnitude.speedValue = std::lround(speed * 100.0);
    polar.velocityMagnitude.speedConfidence = Vanetza_ITS2_SpeedConfidence_unavailable;
    // Encoded exactly like artery::CpService (radians * 10) so the ghost cannot be told apart by format.
    polar.velocityDirection.value = std::lround(Geodesy::wrap360(gridHeadingDeg) * ConstantsWGS84::DEG_TO_RAD * 10.0);
    polar.velocityDirection.confidence = Vanetza_ITS2_AngleConfidence_unavailable;

    ASN_SEQUENCE_ADD(&poc->perceivedObjects.list, po);
    poc->numberOfPerceivedObjects = poc->perceivedObjects.list.count;
}

void FaultyCpService::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject* details)
{
    if (signal != m_sigCpmSent || simTime() < m_cpmFaultStartTime || m_cpmFaultMode == "none") {
        return;
    }
    auto cpObj = dynamic_cast<artery::CpObject*>(obj);
    if (!cpObj || !m_vdp) {
        return;
    }
    auto& cpm = const_cast<vanetza::asn1::r2::Cpm&>(cpObj->asn1());
    auto* poc = MessageEditing::perceivedObjects(cpm);
    if (poc) {
        MessageEditing::canonicalOrder(*poc);  // per-object draws must not depend on heap order
    }
    const std::string& mode = m_cpmFaultMode;

    if (mode == "ghost") {
        // a ghost vehicle `magnitude` metres ahead, driving along with the attacker
        const double speed = m_vdp->speed().value();
        const double gridHeading = m_vdp->heading().value() * ConstantsWGS84::RAD_TO_DEG;  // clockwise from grid north
        double dx = 0.0, dy = 0.0;
        gridDisplacement(gridHeading, m_cpmFaultMagnitude, dx, dy);
        injectGhost(cpm, dx, dy, speed, gridHeading);
        EvaluationTruth::markFalsifiedObject(cpm, m_ghostId);
    } else if (poc && poc->perceivedObjects.list.count > 0) {
        // VeReMi Extension position malfunctions (F2MD parameters) transposed to every perceived object
        if (mode == "object_const_offset") {
            shiftObjects(*poc, [this]() { return m_constOffset; });
        } else {
            shiftObjects(*poc, [this]() { return F2md::randomPosOffset([this](double a, double b) { return uniform(a, b); }); });
        }
        EvaluationTruth::markFalsified(cpm);
    }

    std::string error;
    if (!cpm.validate(error)) {
        ++m_invalidCpms;
        EV_WARN << "CPM invalid after fault injection (receivers will drop it): " << error << "\n";
    }
}

} // namespace artery
