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

//// Moves every perceived object by a playground offset [m].
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
    m_cpmFaultStartTime = par("cpmFaultStartTime").doubleValue();
    if (m_cpmFaultMode != "none" && m_cpmFaultMode != "object_const_offset" &&
        m_cpmFaultMode != "object_random_offset") {
        throw cRuntimeError("unknown cpmFaultMode '%s'", m_cpmFaultMode.c_str());
    }
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

    if (poc && poc->perceivedObjects.list.count > 0) {
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
