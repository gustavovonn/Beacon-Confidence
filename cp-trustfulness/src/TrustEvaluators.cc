#include "../include/TrustEvaluators.h"
#include "../include/Constants.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace artery
{

// -----------------------------------------------------------------------------
// Beta Reputation System Evaluator
// -----------------------------------------------------------------------------
double BetaReputationTrustEvaluator::evaluateTrust(const TrustEvaluationContext& context,
                                                   const TrustEvaluationParams& params)
{
    // Convert current trust score [0,1] to pseudo Beta parameters (alpha, beta)
    // Filter inertia (effective memory horizon): N_eff = 1 / (1 - λ)
    double priorWeight = (params.priorWeight > 0.0) ? params.priorWeight 
                                                    : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;
    double alpha = context.lastTrustScore * priorWeight + 1.0;
    double beta = (1.0 - context.lastTrustScore) * priorWeight + 1.0;

    // --- 1. Hard Physical Invariance Verification (Pre-computed by caller) ---
    const PhysicalInvarianceResult& physicsResult = context.physicsResult;

    // --- 2. Context-Aware Dynamic Threshold ---
    // Applying offline optimized weights with sanitized speed:
    // dynamicMaxPosDev = base (5.0) + speedWeight*(safeSpeed*dt) + yawWeight*(yawRate*safeSpeed)
    double dynamicMaxPosDev = params.maxPosDev 
                            + params.contextSpeedWeight * (physicsResult.safeSpeed * physicsResult.dt) 
                            + params.contextYawWeight * (std::abs(context.pastState.yawRate) * physicsResult.safeSpeed);

    // --- 3. Update Beta Coefficients ---
    if (physicsResult.failed) {
        // Direct physical law violation: PENALIZE IMMEDIATELY.
        // DO NOT allow Phase 2 Centroid Check to forgive this!
        double betaPenalty = std::min(params.maxBetaPenalty, 
                                      params.penaltySevereBase + params.penaltySevereScale * physicsResult.excessRatio);
        beta += betaPenalty;
    } else if (context.distError <= dynamicMaxPosDev) {
        alpha += params.alphaReward;
    } else {
        // Prediction failed, but telemetry speed matches physical displacement.
        // Legitimate abrupt maneuver (e.g. sharp turn or emergency brake) vs teleportation.
        // Calculate the actual time span from the oldest history packet to now
        double actualTimeWindow = 0.1;
        if (!context.history.empty()) {
            actualTimeWindow = context.currentState.time - context.history.front().time;
        }

        double expectedCentroidLimit = (context.currentState.speed * actualTimeWindow * params.centroidSpeedFactor) + dynamicMaxPosDev;
        
        bool centroidCheckPassed = (!context.history.empty() && context.centroidDeviation <= expectedCentroidLimit);
        
        if (centroidCheckPassed) {
            // Saved by Phase 2 Centroid Check!
            // The car is physically where it should be relative to its recent trajectory.
            alpha += params.alphaManeuverReward;
        } else if (context.distError <= dynamicMaxPosDev * params.softMarginRatio) {
            // Soft margin buffer for minor kinematic/viaduct jitter
            double excessRatio = (context.distError - dynamicMaxPosDev) / dynamicMaxPosDev;
            double betaPenalty = params.penaltySoftBase + params.penaltySoftScale * excessRatio;
            beta += betaPenalty;
        } else {
            // Severe discrepancy (> softMarginRatio overshoot, e.g. 20m - 500m fault)
            // A teleportation attack drops trust < 0.5 within ETSI TS 103 759 reaction limits.
            double excessRatio = (context.distError - dynamicMaxPosDev) / dynamicMaxPosDev;
            double betaPenalty = std::min(params.maxBetaPenalty, 
                                          params.penaltySevereBase + params.penaltySevereScale * excessRatio);
            beta += betaPenalty;
        }
    }

    double newTrust = alpha / (alpha + beta);
    return std::max(0.0, std::min(1.0, newTrust));
}

// -----------------------------------------------------------------------------
// Factory Implementation
// -----------------------------------------------------------------------------
std::unique_ptr<ITrustEvaluator> TrustEvaluatorFactory::create(const std::string& name)
{
    // In this paper we will focus only in BetaReputation
    return std::make_unique<BetaReputationTrustEvaluator>();
}

} // namespace artery
