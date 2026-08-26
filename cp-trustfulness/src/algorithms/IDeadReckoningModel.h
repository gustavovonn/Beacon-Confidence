#ifndef I_DEAD_RECKONING_MODEL_H_
#define I_DEAD_RECKONING_MODEL_H_

#include <omnetpp/simtime.h>
#include <string>
#include <deque>

namespace artery
{

struct KinematicState {
    double lat = 0.0;
    double lon = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    omnetpp::SimTime time;
};

struct MotionVector {
    // TODO: MotionVector contém KinematicState(?)
    // TODO: remover stationId de MotionVector
    long stationId = 0;
    double lat = 0.0;      // in degrees
    double lon = 0.0;      // in degrees
    double speed = 0.0;    // in m/s
    double heading = 0.0;  // in degrees from North
    double accel = 0.0;    // in m/s^2
    double yawRate = 0.0;  // in rad/s
    omnetpp::SimTime time;

    double trustScore = 1.0;
    
    double predictionError = 0.0;
    double centroidDeviation = 0.0;
};

class IDeadReckoningModel
{
public:
    virtual ~IDeadReckoningModel() = default;

    // Predict vehicle state forward over delta time (dt in seconds)
    virtual void predict(std::deque<KinematicState> history, const MotionVector& motionVector, KinematicState& nextState, double dt) = 0;

    // Compute spatial position error in meters between actual received CAM and predicted expected state
    virtual double computeError(const MotionVector& state, double actualLat, double actualLon,
                                double actualSpeed, double actualHeading) = 0;

    virtual std::string getName() const = 0;
};

} // namespace artery

#endif // I_DEAD_RECKONING_MODEL_H_
