#ifndef I_DEAD_RECKONING_MODEL_H_
#define I_DEAD_RECKONING_MODEL_H_

#include <omnetpp/simtime.h>
#include <string>
#include <deque>

namespace artery
{

struct CamSnapshot {
    double lat = 0.0;
    double lon = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    omnetpp::SimTime time;
};

struct VehicleState {
    long stationId = 0;
    double lastLat = 0.0;      // in degrees
    double lastLon = 0.0;      // in degrees
    double lastSpeed = 0.0;    // in m/s
    double lastHeading = 0.0;  // in degrees from North
    double lastAccel = 0.0;    // in m/s^2
    double lastYawRate = 0.0;  // in rad/s
    omnetpp::SimTime lastCamTime;

    double expectedLat = 0.0;
    double expectedLon = 0.0;
    double expectedSpeed = 0.0;
    double expectedHeading = 0.0;

    double trustScore = 1.0;
    
    std::deque<CamSnapshot> history;
    double lastPredictionError = 0.0;
    double lastCentroidDeviation = 0.0;
};

class IDeadReckoningModel
{
public:
    virtual ~IDeadReckoningModel() = default;

    // Predict vehicle state forward over delta time (dt in seconds)
    virtual void predict(VehicleState& state, double dt) = 0;

    // Compute spatial position error in meters between actual received CAM and predicted expected state
    virtual double computeError(const VehicleState& state, double actualLat, double actualLon,
                                double actualSpeed, double actualHeading) = 0;

    virtual std::string getName() const = 0;
};

} // namespace artery

#endif // I_DEAD_RECKONING_MODEL_H_
