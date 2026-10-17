#ifndef I_DEAD_RECKONING_MODEL_H_
#define I_DEAD_RECKONING_MODEL_H_

#include <string>
#include <deque>

struct PredictedState {
    double lat = 0.0;      
    double lon = 0.0;     
    double speed = 0.0;
    double heading = 0.0;
};

struct KinematicState {
    double lat = 0.0;      // in degrees
    double lon = 0.0;      // in degrees
    double speed = 0.0;    // in m/s
    double heading = 0.0;  // in degrees from North
    double accel = 0.0;    // in m/s^2
    double yawRate = 0.0;  // in rad/s

    double time = 0.0;
};

class IDeadReckoningModel
{
public:
    virtual ~IDeadReckoningModel() = default;

    // Predict vehicle state forward over delta time (dt in seconds)
    virtual void predict(const std::deque<KinematicState>& history, 
                         const KinematicState& lastState, 
                         PredictedState& nextState, 
                         double dt) = 0;

    // Compute spatial position error in meters between actual received CAM and predicted expected state
    virtual double computeError(const PredictedState& expected, 
                                const KinematicState& actual) = 0;

    virtual std::string getName() const = 0;
};

#endif // I_DEAD_RECKONING_MODEL_H_
