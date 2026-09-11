#ifndef I_TRUST_EVALUATOR_H_
#define I_TRUST_EVALUATOR_H_

#include "IDeadReckoningModel.h"
#include "Constants.h"
#include <string>
#include <deque>

namespace artery
{

struct TrustEvaluationParams {
    // Context-Aware Dynamic Threshold parameters
    double maxPosDev = 0.0;       // meters (acts as baseline intercept)
    double contextSpeedWeight = 0.0;
    double contextYawWeight = 0.0;

    // Reputation Filter Dynamics (N_eff = 1 / (1 - λ))
    double priorWeight = ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;

    // Speed Discrepancy Tolerance (|v_reported - v_implied|) [m/s]
    double speedToleranceMps = ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS;

    // Reward Policy (Evidence increments)
    double alphaReward = ConstantsPhysics::DEFAULT_ALPHA_REWARD;
    double alphaManeuverReward = ConstantsPhysics::DEFAULT_ALPHA_MANEUVER_REWARD;

    // Penalty Policy (Severity-graduated increments)
    double softMarginRatio = ConstantsPhysics::DEFAULT_SOFT_MARGIN_RATIO;
    double penaltySoftBase = ConstantsPhysics::DEFAULT_PENALTY_SOFT_BASE;
    double penaltySoftScale = ConstantsPhysics::DEFAULT_PENALTY_SOFT_SCALE;
    double penaltySevereBase = ConstantsPhysics::DEFAULT_PENALTY_SEVERE_BASE;
    double penaltySevereScale = ConstantsPhysics::DEFAULT_PENALTY_SEVERE_SCALE;
    double maxBetaPenalty = ConstantsPhysics::DEFAULT_MAX_BETA_PENALTY;
    double centroidSpeedFactor = ConstantsPhysics::DEFAULT_CENTROID_SPEED_FACTOR;
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
