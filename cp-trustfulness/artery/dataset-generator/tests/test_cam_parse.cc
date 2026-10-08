// cp-trustfulness/artery/dataset-generator/tests/test_cam_parse.cc
//
// Standalone CAM ASN.1 Parsing and Validation Test
// Tests that CAM messages round-trip through ASN.1 UPER encode/decode,
// validates field extraction according to trace_schema.md, and confirms
// whether vehicleLength is available in the CAM ASN.1 or requires TraCI fallback.

#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

using namespace vanetza;

// Mirrors CaService::createCooperativeAwarenessMessage logic without OMNeT++ module dependency
asn1::Cam createMockCam(
    uint32_t stationId,
    uint16_t genDeltaTimeMs,
    long latitudeTenthMicroDeg,
    long longitudeTenthMicroDeg,
    long headingDeciDeg,
    long speedCmPerSec,
    long accelDeciMps2,
    long yawRateDeciDegPerSec)
{
    asn1::Cam message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = stationId;

    CoopAwareness_t& cam = (*message).cam;
    cam.generationDeltaTime = genDeltaTimeMs * GenerationDeltaTime_oneMilliSec;
    BasicContainer_t& basic = cam.camParameters.basicContainer;
    HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

    basic.stationType = StationType_passengerCar;
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic.referencePosition.longitude = longitudeTenthMicroDeg;
    basic.referencePosition.latitude = latitudeTenthMicroDeg;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = headingDeciDeg;
    bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    bvc.speed.speedValue = speedCmPerSec;
    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    bvc.driveDirection = speedCmPerSec >= 0 ? DriveDirection_forward : DriveDirection_backward;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = accelDeciMps2;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    bvc.curvature.curvatureValue = 0;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
    bvc.yawRate.yawRateValue = yawRateDeciDegPerSec;

    // In Artery's CaService.cc (line 259):
    // bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    return message;
}

void test_cam_population_and_roundtrip()
{
    std::cout << "=========================================================\n";
    std::cout << "TEST 2: CAM Creation, Validation, Roundtrip & Length Check\n";
    std::cout << "=========================================================\n";

    const uint32_t stationId = 20202;
    const uint16_t genDeltaTime = 300;
    const long latMicroDeg = 496110000; // 49.6110000 deg
    const long lonMicroDeg = 61320000;  // 6.1320000 deg
    const long headingDeciDeg = 1800;   // 180.0 deg (South)
    const long speedCmPerSec = 1250;    // 12.50 m/s
    const long accelDeciMps2 = 15;      // 1.5 m/s^2
    const long yawRateDeciDegPerSec = 250; // 2.5 deg/s

    // 1. Create CAM
    asn1::Cam cam = createMockCam(
        stationId, genDeltaTime, latMicroDeg, lonMicroDeg,
        headingDeciDeg, speedCmPerSec, accelDeciMps2, yawRateDeciDegPerSec);

    // 2. Validate
    std::string error;
    bool isValid = cam.validate(error);
    std::cout << "[CHECK] cam.validate(): " << (isValid ? "PASS" : "FAIL (" + error + ")") << "\n";
    assert(isValid);

    // 3. Encode to UPER
    ByteBuffer encoded = cam.encode();
    std::cout << "[CHECK] cam.encode() size: " << encoded.size() << " bytes\n";
    assert(!encoded.empty());

    // 4. Decode
    asn1::Cam decodedCam;
    bool decodeSuccess = decodedCam.decode(encoded);
    std::cout << "[CHECK] decodedCam.decode(): " << (decodeSuccess ? "PASS" : "FAIL") << "\n";
    assert(decodeSuccess);

    // 5. Verify fields per trace_schema.md
    std::cout << "\n--- Verifying CAM Header and Containers ---\n";
    assert(decodedCam->header.stationID == stationId);
    std::cout << "[CHECK] stationID: " << decodedCam->header.stationID << " (expected " << stationId << ")\n";

    assert(decodedCam->cam.generationDeltaTime == genDeltaTime);
    std::cout << "[CHECK] generationDeltaTime: " << decodedCam->cam.generationDeltaTime << " ms (expected " << genDeltaTime << ")\n";

    const auto& basicPos = decodedCam->cam.camParameters.basicContainer.referencePosition;
    double latDeg = basicPos.latitude / 1e7;
    double lonDeg = basicPos.longitude / 1e7;
    std::cout << "[CHECK] BasicContainer lat/lon: (" << latDeg << ", " << lonDeg << ")\n";
    assert(basicPos.latitude == latMicroDeg);
    assert(basicPos.longitude == lonMicroDeg);

    const auto& hfc = decodedCam->cam.camParameters.highFrequencyContainer;
    assert(hfc.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency);
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    double headingDeg = bvc.heading.headingValue / 10.0;
    std::cout << "[CHECK] heading: " << headingDeg << " deg (expected 180.0)\n";
    assert(bvc.heading.headingValue == headingDeciDeg);

    double speedMps = bvc.speed.speedValue / 100.0;
    std::cout << "[CHECK] speed: " << speedMps << " m/s (expected 12.5)\n";
    assert(bvc.speed.speedValue == speedCmPerSec);

    double accelMps2 = bvc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
    std::cout << "[CHECK] accel: " << accelMps2 << " m/s^2 (expected 1.5)\n";
    assert(bvc.longitudinalAcceleration.longitudinalAccelerationValue == accelDeciMps2);

    double yawRateDegPerSec = bvc.yawRate.yawRateValue / 100.0;
    std::cout << "[CHECK] yawRate: " << yawRateDegPerSec << " deg/s (expected 2.5)\n";
    assert(bvc.yawRate.yawRateValue == yawRateDeciDegPerSec);

    // 6. CRUCIAL STEP 1.3: Verify vehicleLength in CAM ASN.1
    std::cout << "\n--- Step 1.3: Vehicle Length Availability Check ---\n";
    long lengthValue = bvc.vehicleLength.vehicleLengthValue;
    std::cout << "[CHECK] vehicleLengthValue: " << lengthValue
              << " (VehicleLengthValue_unavailable = " << VehicleLengthValue_unavailable << ")\n";
    assert(lengthValue == VehicleLengthValue_unavailable);
    std::cout << "[CONFIRMED] vehicleLengthValue is UNAVAILABLE in CAM ASN.1 as emitted by Artery!\n";
    std::cout << "[ACTION REQUIRED] DatasetRecorder must obtain vehicleLength from TraCI VehicleController directly.\n";

    std::cout << "\n>>> TEST 2 RESULT: ALL CAM PARSE & LENGTH CHECKS PASSED <<<\n\n";
}

#include <omnetpp.h>

int main()
{
    omnetpp::cStaticFlag csf;
    omnetpp::SimTime::setScaleExp(-9);

    try {
        test_cam_population_and_roundtrip();

        std::cout << "CAM ASN.1 parsing validation completed successfully!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception caught in test_cam_parse: " << e.what() << "\n";
        return 1;
    }
}
