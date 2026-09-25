#ifndef DEAD_RECKONING_MODELS_H_
#define DEAD_RECKONING_MODELS_H_

#include "IDeadReckoningModel.h"
#include "KinematicRegression.h"
#include "CoordinatePolicies.h"
#include <memory>
#include <string>
#include <deque>

namespace artery
{

template <typename CoordPolicy>
class CtraDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(const std::deque<KinematicState>& history, 
                 const KinematicState& lastState, 
                 PredictedState& nextState, 
                 double dt) override
    {
        if (dt <= 0.0) return;
        CoordPolicy::predictState(lastState, lastState.accel, lastState.yawRate, dt, nextState);
    }

    double computeError(const PredictedState& expected, 
                        const KinematicState& actual) override
    {
        return CoordPolicy::distance(expected.lat, expected.lon, actual.lat, actual.lon);
    }

    std::string getName() const override { return std::string(CoordPolicy::name()) + "CTRA"; }
};

template <typename CoordPolicy>
class HistoryAwareCtraModel : public IDeadReckoningModel
{
public:
    void predict(const std::deque<KinematicState>& history, 
                 const KinematicState& lastState, 
                 PredictedState& nextState, 
                 double dt) override
    {
        if (dt <= 0.0) return;
        double a_hat = lastState.accel;
        double w_hat = lastState.yawRate;
        KinematicRegression::fitOLS(history, lastState.accel, lastState.yawRate, a_hat, w_hat);
        CoordPolicy::predictState(lastState, a_hat, w_hat, dt, nextState);
    }

    double computeError(const PredictedState& expected, 
                        const KinematicState& actual) override
    {
        return CoordPolicy::distance(expected.lat, expected.lon, actual.lat, actual.lon);
    }

    std::string getName() const override { return std::string(CoordPolicy::name()) + "HistoryAwareCTRA"; }
};

// Aliases
using CartesianCtraModel             = CtraDeadReckoningModel<CartesianCoordinatePolicy>;
using GeographicCtraModel            = CtraDeadReckoningModel<Wgs84CoordinatePolicy>;
using CartesianHistoryAwareCtraModel = HistoryAwareCtraModel<CartesianCoordinatePolicy>;
using GeographicHistoryAwareCtraModel = HistoryAwareCtraModel<Wgs84CoordinatePolicy>;

// Backwards compatibility alias
using CtraModel = GeographicCtraModel;

class DeadReckoningModelFactory
{
public:
    static std::unique_ptr<IDeadReckoningModel> create(
        const std::string& name,
        const std::string& coordSystem = "wgs84");
};

} // namespace artery

#endif // DEAD_RECKONING_MODELS_H_
