
import artery.application.ItsG5Service;


//
// CollectivePerceptionService generates  CPMs, i.e. without ASN.1 encoding.
// These  CPMs include the list of detected objects and field-of-view containers
// of the equipped sensors.
//
simple CollectivePerceptionService like ItsG5Service
{
    parameters:
        @signal[CpmSent](type=CpObject);
        @signal[CpmReceived](type=CpObject);

        @statistic[reception](source=CpmReceived;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        double sensorRange = default(100.0);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        bool generateAfterCam = default(false);
        double cpmInterval @unit(s) = default(0.1s);
        double cpmOffset @unit(s) = default(0.05s); // offset to trigger event (CAM or middleware)
        double fovInterval @unit(s) = default(1s);

        int dccProfile = default(2);
        int lengthHeader @unit(byte) = default(37B);
        int lengthFovContainer @unit(byte) = default(9B);
        int lengthObjectContainer @unit(byte) = default(29B);

        string identityRegistryModule;
}
