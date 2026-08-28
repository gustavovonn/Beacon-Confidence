#include "../include/TrustEvaluators.h"
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
    double totalObservations = 10.0;
    double alpha = context.lastTrustScore * totalObservations + 1.0;
    double beta = (1.0 - context.lastTrustScore) * totalObservations + 1.0;

    // --- Context-Aware Dynamic Threshold ---
    // Extract channel congestion (dt)
    double dt = context.currentState.time - context.pastState.time;
    if (dt <= 0.0) dt = 0.1;
    
    // Applying offline optimized weights:
    // dynamicMaxPosDev = base (5.0) + speedWeight*(speed*dt) + yawWeight*(yawRate*speed)
    double dynamicMaxPosDev = params.maxPosDev 
                            + params.contextSpeedWeight * (context.currentState.speed * dt) 
                            + params.contextYawWeight * (std::abs(context.pastState.yawRate) * context.currentState.speed);

    // --- Update Beta Coefficients ---
    if (context.distError <= dynamicMaxPosDev) {
        alpha += 1.0;
    } else {
        // Prediction failed. Is it just a sharp turn or a spatial spoof (teleportation)?
        // Checkinkg Centroid Deviation
        // Calculate the actual time span from the oldest history packet to now
        double actualTimeWindow = 0.1;
        if (!context.history.empty()) {
            actualTimeWindow = context.currentState.time - context.history.front().time;
        }

        // factor 0.75 is just a geometric heuristic: 0.5 lateral max dev + 0.25 overshoot
        double expectedCentroidLimit = (context.currentState.speed * actualTimeWindow * 0.75) + dynamicMaxPosDev;
        
        bool centroidCheckPassed = (!context.history.empty() && context.centroidDeviation <= expectedCentroidLimit);
        

        if (centroidCheckPassed) {
            // Saved by Phase 2 Centroid Check!.
            // The car is physically where it should be relative to its recent trajectory.
            alpha += 0.5; // Small reward, no beta penalty
        } else if (context.distError <= dynamicMaxPosDev * 1.35) {
            // Soft margin buffer for minor kinematic/viaduct jitter (<35% overshoot)
            double excessRatio = (context.distError - dynamicMaxPosDev) / dynamicMaxPosDev;
            double betaPenalty = 0.10 + 0.50 * excessRatio; // Small penalty (0.10 to 0.27) instead of instant >1.0
            beta += betaPenalty;
        } else {
            // Severe discrepancy (>35% overshoot, e.g. 20m - 500m fault)
            // A teleportation attack must be able to drop trust < 0.5 in a single frame.
            // Since alpha = 11 for perfect trust, beta penalty needs to be > 11 to trigger a ban.
            double excessRatio = (context.distError - dynamicMaxPosDev) / dynamicMaxPosDev;
            double betaPenalty = std::min(15.0, 5.0 + 5.0 * excessRatio);
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
