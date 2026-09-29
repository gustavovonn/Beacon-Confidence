package config.neds;

import artery.application.ItsG5Service;

//
// Positioning and perception errors of this station, from cited models (cp-trustfulness/include/ErrorModels.h).
// The same values parameterise the receiver's gates (CollectiveTrustService reads them from here).
//
simple SensorNoiseService like ItsG5Service
{
    parameters:
        @class(artery::SensorNoiseService);
        @display("i=block/cogwheel");

        // GNSS: VeReMi Extension error model [Kamel et al., IEEE ICC 2020, Sec. IV-B]
        string gnssErrorModel = default("veremi");        // "veremi" or "none"
        double gnssPositionBound @unit(m) = default(5m);  // E0 ~ U(-5, 5) m per axis
        double gnssJitterFraction = default(0.03);        // E(t) ~ N((E0 + E(t-1)) / 2, (0.03 E0)^2)
        double gnssHeadingBound = default(20);            // [deg] E0H ~ U(-20, 20)
        double gnssHeadingSpeedDecay = default(0.1);      // [s/m] heading error E0H * exp(-0.1 v)
        double gnssSpeedSigma = default(0.00016);         // speed V (1 + E0V), E0V ~ N(0, 0.00016^2)

        // Radar noise: Jacobs et al. [IEEE VTC2023-Spring]
        string radarErrorModel = default("jacobs");       // "jacobs" or "none"
        double radarRangeSigma @unit(m) = default(0.4m);
        double radarAzimuthSigma = default(3.0);          // [deg] (0.052 rad)
        double radarSpeedSigma = default(0.2);            // [m/s]

        // Field of view: Recommendation ITU-R M.2057-1 (2018), Radar B (high resolution, up to 100 m, receive
        // 10 dB beamwidth +-25 deg), front and rear bumper of a 5 m car. Must match omnetpp.ini (FrontRadar/RearRadar).
        string sensorCoverage = default("100:50:0:2.5,100:50:180:-2.5");  // range:aperture:mountYaw:mountForward
}
