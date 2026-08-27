#ifndef DEAD_RECKONING_MODELS_H_
#define DEAD_RECKONING_MODELS_H_

#include "IDeadReckoningModel.h"
#include <memory>
#include <string>

namespace artery
{

class CtraDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(const std::deque<KinematicState>& history, 
                 const KinematicState& lastState, 
                 PredictedState& nextState, 
                 double dt) override;

    double computeError(const PredictedState& expected, 
                        const KinematicState& actual) override;

    std::string getName() const override { return "CTRA"; }
};

class HistoryAwareCtraModel : public IDeadReckoningModel
{
public:
    void predict(const std::deque<KinematicState>& history, 
                 const KinematicState& lastState, 
                 PredictedState& nextState, 
                 double dt) override;

    double computeError(const PredictedState& expected, 
                        const KinematicState& actual) override;

    std::string getName() const override { return "HistoryAwareCTRA"; }
private:
    double unwrapHeading(double currentHeading, double previousHeading);
};

class DeadReckoningModelFactory
{
public:
    static std::unique_ptr<IDeadReckoningModel> create(const std::string& name);
};

} // namespace artery

#endif // DEAD_RECKONING_MODELS_H_
