#include "../include/TrustEvaluators.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace artery
{

// -----------------------------------------------------------------------------
// Hard Physical Invariance Verification
// -----------------------------------------------------------------------------
PhysicalInvarianceResult BetaReputationTrustEvaluator::checkPhysicalInvariants(const TrustEvaluationContext& context) const
{
    PhysicalInvarianceResult res;

    double dt_raw = context.currentState.time - context.pastState.time;
    res.dt = (dt_raw <= 0.0) ? 0.1 : dt_raw;

    double displacement = std::hypot(context.currentState.lon - context.pastState.lon,
                                     context.currentState.lat - context.pastState.lat);
    double impliedSpeed = displacement / res.dt;

    const double SPEED_TOLERANCE_MPS = 4.0; // 4.0 m/s (~14.4 km/h), covers 99.5% of honest maneuvers
    double speedDiscrepancy = 0.0;

    // Only evaluate inter-frame physical invariance on valid consecutive transmissions:
    // (1) dt_raw >= 0.2s: ignores duplicate receptions of the same broadcast packet across multiple receiver logs
    // (2) dt_raw <= 2.0s: ignores large communication dropouts/gaps (matching F2MD MAX_TIME_DELTA)
    if (dt_raw >= 0.2 && dt_raw <= 2.0) {
        double avgReportedSpeed = 0.5 * (context.currentState.speed + context.pastState.speed);
        speedDiscrepancy = std::abs(avgReportedSpeed - impliedSpeed);
        bool speedFalsified = (speedDiscrepancy > SPEED_TOLERANCE_MPS);

        // Frozen Position: node reports motion (>2.0 m/s) but stayed motionless (<0.3m) over a full dt >= 0.2s
        bool frozenPosActive = (avgReportedSpeed > 2.0 && displacement < 0.3);

        if (speedFalsified || frozenPosActive) {
            res.failed = true;
            res.excessRatio = (speedDiscrepancy > SPEED_TOLERANCE_MPS) ? 
                ((speedDiscrepancy - SPEED_TOLERANCE_MPS) / SPEED_TOLERANCE_MPS) : 1.0;
        }
    }

    // Sanitize speed used in dynamic threshold calculation so attacker cannot inflate bounds
    res.safeSpeed = std::min(context.currentState.speed, impliedSpeed + SPEED_TOLERANCE_MPS);

    return res;
}

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

    // --- 1. Hard Physical Invariance Verification ---
    PhysicalInvarianceResult physicsResult = checkPhysicalInvariants(context);

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
        double betaPenalty = std::min(15.0, 5.0 + 5.0 * physicsResult.excessRatio);
        beta += betaPenalty;
    } else if (context.distError <= dynamicMaxPosDev) {
        alpha += 1.0;
    } else {
        // Prediction failed, but telemetry speed matches physical displacement.
        // Legitimate abrupt maneuver (e.g. sharp turn or emergency brake) vs teleportation.
        // Calculate the actual time span from the oldest history packet to now
        double actualTimeWindow = 0.1;
        if (!context.history.empty()) {
            actualTimeWindow = context.currentState.time - context.history.front().time;
        }

        // factor 0.75 is just a geometric heuristic: 0.5 lateral max dev + 0.25 overshoot
        double expectedCentroidLimit = (context.currentState.speed * actualTimeWindow * 0.75) + dynamicMaxPosDev;
        
        bool centroidCheckPassed = (!context.history.empty() && context.centroidDeviation <= expectedCentroidLimit);
        
        if (centroidCheckPassed) {
            // Saved by Phase 2 Centroid Check!
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
