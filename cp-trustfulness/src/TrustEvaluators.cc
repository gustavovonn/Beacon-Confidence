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

    // --- 2. Update Beta Coefficients ---
    // All pre-checks (dynamic threshold, centroid) are pre-computed by the caller
    // via CheckerPipeline.h and passed in context. Scorer consumes results only.
    if (physicsResult.failed) {
        // Phase 0: Direct physical law violation — penalize immediately.
        // The centroid check CANNOT forgive a Phase 0 failure.
        double betaPenalty = std::min(params.maxBetaPenalty, 
                                      params.penaltySevereBase + params.penaltySevereScale * physicsResult.excessRatio);
        beta += betaPenalty;
    } else if (context.distError <= context.dynamicMaxPosDev) {
        // Phase 1 pass: trajectory within dynamic threshold.
        alpha += params.alphaReward;
    } else if (context.centroidCheckPassed) {
        // Phase 2 rescue: Phase 1 failed but centroid check forgives as legitimate maneuver.
        alpha += params.alphaManeuverReward;
    } else if (context.distError <= context.dynamicMaxPosDev * params.softMarginRatio) {
        // Soft margin: minor kinematic/GNSS jitter just beyond the dynamic threshold.
        double excessRatio = (context.distError - context.dynamicMaxPosDev) / context.dynamicMaxPosDev;
        double betaPenalty = params.penaltySoftBase + params.penaltySoftScale * excessRatio;
        beta += betaPenalty;
    } else {
        // Severe discrepancy (> softMarginRatio overshoot, e.g. 20m - 500m fault)  
        // A teleportation attack drops trust < 0.5 within ETSI TS 103 759 reaction limits.
        double excessRatio = (context.distError - context.dynamicMaxPosDev) / context.dynamicMaxPosDev;
        double betaPenalty = std::min(params.maxBetaPenalty, 
                                      params.penaltySevereBase + params.penaltySevereScale * excessRatio);
        beta += betaPenalty;
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
