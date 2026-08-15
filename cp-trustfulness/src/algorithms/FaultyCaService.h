#ifndef FAULTY_CA_SERVICE_H_
#define FAULTY_CA_SERVICE_H_

#include <artery/application/CaService.h>

namespace artery
{

class FaultyCaService : public CaService
{
public:
    void initialize() override;
    void trigger() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signal, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    omnetpp::simsignal_t m_sigCamSent;
    std::string m_faultMode;
    double m_faultMagnitude;
    omnetpp::SimTime m_faultStartTime;
};

} // namespace artery

#endif // FAULTY_CA_SERVICE_H_
