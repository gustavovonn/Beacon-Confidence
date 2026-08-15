package src.modules;

import artery.application.CaService;

//
// Malicious CA Service that injects faults into outgoing CAMs
//
simple FaultyCaService extends CaService
{
    parameters:
        @class(artery::FaultyCaService);
        
        string faultMode = default("constant_offset");
        double faultMagnitude @unit(m) = default(10m);
        double faultStartTime @unit(s) = default(0s);
}
