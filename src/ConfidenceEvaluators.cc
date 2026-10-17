#include "ConfidenceEvaluators.h"
#include "MechanismConstants.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

// -----------------------------------------------------------------------------
// Beta Reputation System Evaluator
// -----------------------------------------------------------------------------
double BetaReputationEvaluator::evaluateConfidence(const ConfidenceContext& context,
                                                   const EngineConfig& params)
{
    double priorWeight = (params.priorWeight > 0.0) ? params.priorWeight 
                                                    : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;

    double alpha = context.lastConfidence * priorWeight + 1.0;
    double beta = (1.0 - context.lastConfidence) * priorWeight + 1.0;

    // --- Update Beta Coefficients ---
    if (context.physicsResult.failed) {
        // Consistency check failed: direct physical law violation, penalize immediately.
        double betaPenalty = std::min(params.maxBetaPenalty, 
                                      params.penaltySevereBase + params.penaltySevereScale * context.physicsResult.excessRatio);
        beta += betaPenalty;
    } else if (context.distError <= context.dynamicMaxPosDev) {
        // Predictive check passed: trajectory within the dynamic threshold.
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
double EwmaEvaluator::evaluateConfidence(const ConfidenceContext& context,
                                         const EngineConfig& params)
{
    // Filter inertia mapped from priorWeight: λ = priorWeight / (priorWeight + 1)
    double priorWeight = (params.priorWeight > 0.0) ? params.priorWeight 
                                                    : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;
    double lambda = priorWeight / (priorWeight + 1.0);

    // Instantaneous evidence observation in [0, 1]
    double obs = 0.0;
    if (context.physicsResult.failed) {
        // Consistency check failed: hard physics violation
        obs = 0.0;
    } else if (context.distError <= context.dynamicMaxPosDev) {
        // Predictive check passed: full agreement
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

    double newTrust = lambda * context.lastConfidence + (1.0 - lambda) * obs;
    return std::max(0.0, std::min(1.0, newTrust));
}

// -----------------------------------------------------------------------------
// Sliding Window Majority Vote Evaluator
// -----------------------------------------------------------------------------
double MajorityVoteEvaluator::evaluateConfidence(const ConfidenceContext& context,
                                                 const EngineConfig& params)
{
    (void)params;
    // Binary verdict: pass if the consistency check held AND the predictive check passed
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
std::unique_ptr<IConfidenceEvaluator> ConfidenceEvaluatorFactory::create(const std::string& name,
                                                               size_t windowSize)
{
    if (name == "EWMA" || name == "ewma") {
        return std::make_unique<EwmaEvaluator>();
    } else if (name == "MajorityVote" || name == "majority_vote" || name == "majority") {
        return std::make_unique<MajorityVoteEvaluator>(windowSize);
    }
    return std::make_unique<BetaReputationEvaluator>();
}
