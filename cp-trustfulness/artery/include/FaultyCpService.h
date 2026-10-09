#ifndef FAULTY_CP_SERVICE_H_
#define FAULTY_CP_SERVICE_H_

#include <artery/envmod/service/CpService.h>
#include "../../include/Geodesy.h"
#include <cstdint>

namespace artery
{

class VehicleDataProvider;

/// CP service of a faulty station: VeReMi's position malfunctions applied to every perceived object.
class FaultyCpService : public CpService
{
public:
    void initialize() override;
    void finish() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signal, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    omnetpp::simsignal_t m_sigCpmSent;
    const VehicleDataProvider* m_vdp = nullptr;
    std::string m_cpmFaultMode;
    omnetpp::SimTime m_cpmFaultStartTime;
    LocalPoint m_constOffset;  ///< object_const_offset [m east/north]
    unsigned long m_invalidCpms = 0;
};

} // namespace artery

#endif // FAULTY_CP_SERVICE_H_
