package config.neds;

import artery.envmod.service.CpService;

//
// CP service of a faulty station: falsifies the content of outgoing CPMs.
//   none                  honest behaviour
//   ghost                 a moving ghost vehicle cpmFaultMagnitude ahead of the station (attack)
//   object_const_offset   every perceived object shifted by one constant offset (VeReMi ConstPosOffset, F2MD values)
//   object_random_offset  every perceived object shifted by a new random offset (VeReMi RandomPosOffset, F2MD values)
//
simple FaultyCpService extends CpService
{
    parameters:
        @class(artery::FaultyCpService);
        string cpmFaultMode = default("ghost");
        double cpmFaultMagnitude @unit(m) = default(20m);  // ghost distance ahead
        double cpmFaultStartTime @unit(s) = default(10s);
}
