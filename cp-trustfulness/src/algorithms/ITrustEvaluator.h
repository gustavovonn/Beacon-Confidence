#ifndef I_TRUST_EVALUATOR_H_
#define I_TRUST_EVALUATOR_H_

#include "IDeadReckoningModel.h"
#include <string>
#include <deque>

namespace artery
{

struct TrustEvaluationParams {
    // Context-Aware Dynamic Threshold parameters
    double maxPosDev = 0.0;       // meters (acts as baseline intercept)
    double contextSpeedWeight = 0.0;
    double contextYawWeight = 0.0;
};

struct TrustEvaluationContext {
    const std::deque<KinematicState>& history;
    const KinematicState& pastState;
    const KinematicState& currentState;
    double lastTrustScore;
    double distError;
    double centroidDeviation;
};

class ITrustEvaluator
{
public:
    virtual ~ITrustEvaluator() = default;

    // Evaluate and return updated trust score in range [0.0, 1.0]
    virtual double evaluateTrust(const TrustEvaluationContext& context,
                                 const TrustEvaluationParams& params) = 0;

    virtual std::string getName() const = 0;
};

} // namespace artery

#endif // I_TRUST_EVALUATOR_H_
