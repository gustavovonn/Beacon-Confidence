package config.neds;

import artery.application.CaService;

//
// CA service of a faulty station.
//   none                 honest behaviour
//   VeReMi Extension speed malfunctions (F2MD parameters, see include/F2mdFaults.h):
//     const_speed, const_speed_offset, random_speed, random_speed_offset
//   Paper 1 position faults on the CAM only (magnitude and bearing below):
//     constant_offset  fixed offset of faultMagnitude along faultBearing
//     random_noise     uniform error in [-faultMagnitude, +faultMagnitude] per axis, every CAM
//     gaussian_noise   Gaussian error with sigma = faultMagnitude per axis, every CAM
//     gradual_drift    offset along faultBearing growing to faultMagnitude after 10 s
// VeReMi's position malfunctions are positioning failures: FaultySensorNoiseService.gnssFaultMode.
//
simple FaultyCaService extends CaService
{
    parameters:
        @class(artery::FaultyCaService);
        string faultMode = default("constant_offset");
        double faultMagnitude @unit(m) = default(10m);
        double faultBearing = default(-1);              // [deg clockwise from true north]; < 0: random per station
        double faultStartTime @unit(s) = default(0s);
}
