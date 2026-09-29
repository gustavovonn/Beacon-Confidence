#ifndef FAULTY_CA_SERVICE_H_
#define FAULTY_CA_SERVICE_H_

#include <artery/application/CaService.h>
#include <vanetza/asn1/cam.hpp>
#include "../../include/Geodesy.h"

namespace artery
{

class VehicleDataProvider;

/// CA service of a faulty station: one of the four VeReMi Extension speed malfunctions (F2mdFaults.h) on the
/// speed of every outgoing CAM. VeReMi's position malfunctions are positioning failures (FaultySensorNoiseService);
/// Paper 1's CAM-only position faults (constant_offset, random_noise, gaussian_noise, gradual_drift) are kept.
class FaultyCaService : public CaService
{
public:
    void initialize() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signal, omnetpp::cObject* obj,
                       omnetpp::cObject* details) override;

private:
    static bool isPositionMode(const std::string& mode);
    void falsifyPosition(vanetza::asn1::Cam& cam);

    omnetpp::simsignal_t m_sigCamSent;
    const VehicleDataProvider* m_vdp = nullptr;
    std::string m_faultMode;
    omnetpp::SimTime m_faultStartTime;
    double m_faultMagnitude = 0.0;
    double m_faultBearing = 0.0;
    LocalPoint m_constant;  ///< const_speed: the velocity; const_speed_offset: the offset [m/s east/north]
};

} // namespace artery

#endif // FAULTY_CA_SERVICE_H_
