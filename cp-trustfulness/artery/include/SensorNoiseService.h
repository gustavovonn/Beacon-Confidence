#ifndef SENSOR_NOISE_SERVICE_H_
#define SENSOR_NOISE_SERVICE_H_

#include <artery/application/ItsG5BaseService.h>
#include <vanetza/asn1/cpm.hpp>
#include "../../include/ErrorModels.h"
#include "../../include/Geodesy.h"
#include "../../include/SensorCoverage.h"
#include <optional>
#include <string>

namespace traci { class VehicleController; }

namespace artery
{

class GlobalEnvironmentModel;
class IdentityRegistry;
class VehicleDataProvider;

/**
 * @brief Positioning and perception errors of this station, with referenced models (ErrorModels.h).
 *
 * Listens to CamSent and CpmSent of its own station and, before the messages are encoded:
 *   - GNSS (VeReMi Extension): shifts the CAM and CPM reference positions by the station's position error, adds
 *     the heading error to the CAM heading and the CPM orientation, and scales the CAM speed;
 *   - radar (Jacobs et al.): measures every perceived object again from the sensor mount with range and azimuth
 *     noise and adds speed noise. Which objects are detected is Artery's FoV sensor (ITU-R M.2057-1 field of view).
 *
 * A faulty station (FaultySensorNoiseService) may also carry one of the four VeReMi Extension position
 * malfunctions (F2mdFaults.h), applied to CAM and CPM alike, as a positioning failure would.
 *
 * The receiver's own pose uses the same GNSS model (gnssErrorNow()). Attack services modify the messages first
 * (their listener sits on the emitting module).
 */
class SensorNoiseService : public ItsG5BaseService
{
public:
    void initialize() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
    bool requiresListener() const override { return false; }
    void finish() override;

    /// Current position error of this station (nominal error + position malfunction) [m, east/north].
    LocalPoint gnssErrorNow();

    /// Radar measurement of `target`, detected by Artery's FoV sensor, by a vehicle at `hostCentre` heading
    /// `hostHeadingDeg` (true geometry).
    GeoPoint radarMeasure(const GeoPoint& hostCentre, double hostHeadingDeg, const GeoPoint& target);
    /// Measured speed of a detected object [m/s].
    double radarSpeed(double speed);

    /// The models as configured on a SensorNoiseService module (readable before it is initialised).
    static GnssErrorModel gnssModelOf(omnetpp::cModule& noise);
    static RadarErrorModel radarModelOf(omnetpp::cModule& noise);
    static SensorCoverage coverageOf(omnetpp::cModule& noise);

private:
    void updateGnssError();
    LocalPoint malfunctionOffset();
    void shiftReference(long& latitude, long& longitude);
    double headingErrorNow() const;
    void recordObjectStation(const vanetza::asn1::r2::Cpm& cpm, uint16_t objectId, double dx, double dy);

    omnetpp::simsignal_t m_camSent;
    omnetpp::simsignal_t m_cpmSent;
    const VehicleDataProvider* m_vdp = nullptr;
    const traci::VehicleController* m_controller = nullptr;
    GlobalEnvironmentModel* m_gem = nullptr;
    IdentityRegistry* m_identities = nullptr;

    GnssErrorModel m_gnss;
    RadarErrorModel m_radar;
    SensorCoverage m_coverage;
    bool m_gnssOn = true;
    bool m_radarOn = true;

    // VeReMi Extension state of this station
    LocalPoint m_e0;              ///< E0 per axis [m]
    LocalPoint m_error;           ///< E_t [m]
    double m_errorTime = -1.0;
    double m_headingE0 = 0.0;     ///< E0H [deg]
    double m_speedE0 = 0.0;       ///< E0V

    // position malfunction (FaultySensorNoiseService only)
    std::string m_faultMode = "none";
    double m_faultStartTime = 0.0;
    LocalPoint m_faultConstant;   ///< const_pos: the playground point (east/north from the origin); const_pos_offset: the offset
    LocalPoint m_faultSample;
    double m_faultSampleTime = -1.0;
    double m_playgroundWidth = 0.0;
    double m_playgroundHeight = 0.0;

    unsigned long m_invalidCpms = 0;
};

} // namespace artery

#endif // SENSOR_NOISE_SERVICE_H_
