package config.neds;

//
// Positioning of a faulty station: the regular errors plus one of the VeReMi Extension position malfunctions
// (F2MD parameters, include/F2mdFaults.h), applied to the reference position of both CAM and CPM, as a
// positioning failure would:
//   none, const_pos, const_pos_offset, random_pos, random_pos_offset
//
simple FaultySensorNoiseService extends SensorNoiseService
{
    parameters:
        @class(artery::SensorNoiseService);
        string gnssFaultMode = default("none");
        double gnssFaultStartTime @unit(s) = default(10s);
}
