package object_trajectory;

import artery.application.ItsG5Service;

//
// Records an honest trace of CAM and CPM content.
//
// Records only. No error model, no fault, no judgement: those are ingestion-time choices applied
// later, so one recording serves every subset. Perceived objects are written as
// offsets relative to their author, with the author's pose in separate columns.
//
simple ArteryDatasetGenerator like ItsG5Service
{
    parameters:
        @class(artery::ArteryDatasetGenerator);
        @signal[CamReceived];
        @signal[CpmReceived];

        // Where the dataset is written. Every vehicle appends to the same files; message content is
        // deduplicated by message id, so a CAM received by forty neighbours is stored once and
        // reception.csv carries the forty arrivals.
        string outputDir = default("dataset/groundtruth");

        // Discard the ramp-up, during which vehicles are still being inserted and the neighbourhood
        // size is not representative of anything.
        double warmup @unit(s) = default(10s);

        // Used when a CAM omits vehicleLength. Only affects the length column.
        double defaultVehicleLength @unit(m) = default(4.5m);

        // "artery" or "etsi". Artery encodes CPM angles against grid north and offsets in its own
        // delta convention; the standard uses true north and east/north. Getting this wrong rotates
        // every perceived object by the local meridian convergence.
        string cpmEncoding = default("artery");

        @display("i=block/table");
}
