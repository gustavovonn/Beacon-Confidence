#include "FaultyCaService.h"
#include <artery/application/CaObject.h>
#include <cstdlib>

using namespace omnetpp;

namespace artery
{

Define_Module(FaultyCaService)

void FaultyCaService::initialize()
{
    CaService::initialize();
    
    m_faultMode = par("faultMode").stringValue();
    m_faultMagnitude = par("faultMagnitude").doubleValue();
    m_faultStartTime = par("faultStartTime").doubleValue();
    
    m_sigCamSent = cComponent::registerSignal("CamSent");
    cComponent::subscribe(m_sigCamSent, this);

    recordScalar("isFaulty", 1);
}

void FaultyCaService::trigger()
{
    // Pass through to standard CA behavior.
    // The outgoing CAM is intercepted in receiveSignal.
    CaService::trigger();
}

void FaultyCaService::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject* details)
{
    // manipulates right before the emission of the CAM 
    if (signal == m_sigCamSent) {
        if (omnetpp::simTime().dbl() >= m_faultStartTime.dbl()) {
            auto caObj = dynamic_cast<artery::CaObject*>(obj);
            if (caObj) {
                // Intercept and modify the ASN.1 packet just before byte buffer serialization
                vanetza::asn1::Cam& cam = const_cast<vanetza::asn1::Cam&>(caObj->asn1());
                
                // roughly map faultMagnitude (meters) to microdegrees
                // 1 meter is approx 1e5 / 11.132 microdegrees = 8984 microdegrees
                long offsetMicro = static_cast<long>(m_faultMagnitude * 1e5 / 11.132);
                
                if (m_faultMode == "constant_offset" || m_faultMode == "offset" || m_faultMode == "constant") {
                    cam->cam.camParameters.basicContainer.referencePosition.latitude += offsetMicro;
                    cam->cam.camParameters.basicContainer.referencePosition.longitude += offsetMicro;
                } else if (m_faultMode == "random_noise" || m_faultMode == "noise" || m_faultMode == "random") {
                    long noiseLat = (std::rand() % (2 * offsetMicro + 1)) - offsetMicro;
                    long noiseLon = (std::rand() % (2 * offsetMicro + 1)) - offsetMicro;
                    cam->cam.camParameters.basicContainer.referencePosition.latitude += noiseLat;
                    cam->cam.camParameters.basicContainer.referencePosition.longitude += noiseLon;
                } else if (m_faultMode == "gradual_drift" || m_faultMode == "drift" || m_faultMode == "ramp") {
                    // Linear ramp attack: offset grows progressively over time (0.5m/s per second)
                    double elapsedTime = omnetpp::simTime().dbl() - m_faultStartTime.dbl();
                    double driftMeters = m_faultMagnitude * (elapsedTime / 10.0); // scales to faultMagnitude after 10s
                    long driftMicro = static_cast<long>(driftMeters * 1e5 / 11.132);
                    cam->cam.camParameters.basicContainer.referencePosition.latitude += driftMicro;
                    cam->cam.camParameters.basicContainer.referencePosition.longitude += driftMicro;
                }
            }
        }
    }
}

} // namespace artery
