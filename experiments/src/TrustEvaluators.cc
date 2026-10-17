#include "../include/TrustEvaluators.h"
#include "../include/Constants.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

// -----------------------------------------------------------------------------
// Beta Reputation System Evaluator
// -----------------------------------------------------------------------------
double BetaReputationTrustEvaluator::evaluateTrust(const TrustEvaluationContext& context,
                                                   const TrustEvaluationParams& params)
{
    double priorWeight = (params.priorWeight > 0.0) ? params.priorWeight 
                                                    : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;

    double alpha = context.lastTrustScore * priorWeight + 1.0;
    double beta = (1.0 - context.lastTrustScore) * priorWeight + 1.0;

    // --- Update Beta Coefficients ---
    if (context.physicsResult.failed) {
        // Phase 0: Direct physical law violation — penalize immediately.
        double betaPenalty = std::min(params.maxBetaPenalty, 
                                      params.penaltySevereBase + params.penaltySevereScale * context.physicsResult.excessRatio);
        beta += betaPenalty;
    } else if (context.distError <= context.dynamicMaxPosDev) {
        // Phase 1 pass: trajectory within dynamic threshold.
        alpha += params.alphaReward;
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
// EWMA Evaluator
// -----------------------------------------------------------------------------
double EwmaTrustEvaluator::evaluateTrust(const TrustEvaluationContext& context,
                                         const TrustEvaluationParams& params)
{
    // Filter inertia mapped from priorWeight: λ = priorWeight / (priorWeight + 1)
    double priorWeight = (params.priorWeight > 0.0) ? params.priorWeight 
                                                    : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;
    double lambda = priorWeight / (priorWeight + 1.0);

    // Instantaneous evidence observation in [0, 1]
    double obs = 0.0;
    if (context.physicsResult.failed) {
        // Phase 0 hard physics violation
        obs = 0.0;
    } else if (context.distError <= context.dynamicMaxPosDev) {
        // Phase 1 pass: full consistency
        obs = 1.0;
    } else if (context.distError <= context.dynamicMaxPosDev * params.softMarginRatio) {
        // Soft jitter margin: linear degradation
        double excessRatio = (context.distError - context.dynamicMaxPosDev) / context.dynamicMaxPosDev;
        double maxExcess = params.softMarginRatio - 1.0;
        double factor = (maxExcess > 0.0) ? std::max(0.0, 1.0 - excessRatio / maxExcess) : 0.0;
        obs = 0.5 * factor;
    } else {
        // Severe excess
        obs = 0.0;
    }

    double newTrust = lambda * context.lastTrustScore + (1.0 - lambda) * obs;
    return std::max(0.0, std::min(1.0, newTrust));
}

// -----------------------------------------------------------------------------
// Sliding Window Majority Vote Evaluator
// -----------------------------------------------------------------------------
double MajorityVoteTrustEvaluator::evaluateTrust(const TrustEvaluationContext& context,
                                                 const TrustEvaluationParams& params)
{
    (void)params;
    // Binary verdict: pass if no Phase 0 failure AND Phase 1 passed
    bool passed = false;
    if (!context.physicsResult.failed) {
        if (context.distError <= context.dynamicMaxPosDev) {
            passed = true;
        }
    }

    auto& window = m_verdictWindows[context.senderId];
    window.push_back(passed);
    while (window.size() > m_windowSize) {
        window.pop_front();
    }

    if (window.empty()) return 1.0;

    size_t passCount = 0;
    for (bool v : window) {
        if (v) ++passCount;
    }

    return static_cast<double>(passCount) / static_cast<double>(window.size());
}

// -----------------------------------------------------------------------------
// Factory Implementation
// -----------------------------------------------------------------------------
std::unique_ptr<ITrustEvaluator> TrustEvaluatorFactory::create(const std::string& name,
                                                               size_t windowSize)
{
    if (name == "EWMA" || name == "ewma") {
        return std::make_unique<EwmaTrustEvaluator>();
    } else if (name == "MajorityVote" || name == "majority_vote" || name == "majority") {
        return std::make_unique<MajorityVoteTrustEvaluator>(windowSize);
    }
    return std::make_unique<BetaReputationTrustEvaluator>();
}
