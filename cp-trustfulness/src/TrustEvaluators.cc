#include "../include/TrustEvaluators.h"
#include "../include/Constants.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace artery
{

// -----------------------------------------------------------------------------
// Hard Physical Invariance Verification
// -----------------------------------------------------------------------------
PhysicalInvarianceResult BetaReputationTrustEvaluator::checkPhysicalInvariants(const TrustEvaluationContext& context,
                                                                              const TrustEvaluationParams& params) const
{
    PhysicalInvarianceResult res;

    double dt_raw = context.currentState.time - context.pastState.time;
    res.dt = (dt_raw <= 0.0) ? 0.1 : dt_raw;

    double displacement = std::hypot(context.currentState.lon - context.pastState.lon,
                                     context.currentState.lat - context.pastState.lat);
    double impliedSpeed = displacement / res.dt;

    double speedTolerance = (params.speedToleranceMps > 0.0) ? params.speedToleranceMps 
                                                             : ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS;
    double speedDiscrepancy = 0.0;

    // Only evaluate inter-frame physical invariance on valid consecutive transmissions:
    // (1) dt_raw >= MIN_INTER_FRAME_TIME: rejects duplicate receptions across multiple receiver logs (ETSI EN 302 637-2)
    // (2) dt_raw <= MAX_INTER_FRAME_TIME: ignores large communication gaps where kinematics are discontinuous (F2MD MAX_TIME_DELTA)
    if (dt_raw >= ConstantsPhysics::MIN_INTER_FRAME_TIME && dt_raw <= ConstantsPhysics::MAX_INTER_FRAME_TIME) {
        double avgReportedSpeed = 0.5 * (context.currentState.speed + context.pastState.speed);
        speedDiscrepancy = std::abs(avgReportedSpeed - impliedSpeed);
        bool speedFalsified = (speedDiscrepancy > speedTolerance);

        // Frozen Position: node reports motion (> FROZEN_SPEED_THRESHOLD) but stayed motionless (< FROZEN_DISPLACEMENT_MIN)
        bool frozenPosActive = (avgReportedSpeed > ConstantsPhysics::FROZEN_SPEED_THRESHOLD && 
                                displacement < ConstantsPhysics::FROZEN_DISPLACEMENT_MIN);

        if (speedFalsified || frozenPosActive) {
            res.failed = true;
            res.excessRatio = (speedDiscrepancy > speedTolerance) ? 
                ((speedDiscrepancy - speedTolerance) / speedTolerance) : 1.0;
        }
    }

    // Sanitize speed used in dynamic threshold calculation so attacker cannot inflate bounds
    res.safeSpeed = std::min(context.currentState.speed, impliedSpeed + speedTolerance);

    return res;
}

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

    // --- 1. Hard Physical Invariance Verification ---
    PhysicalInvarianceResult physicsResult = checkPhysicalInvariants(context, params);

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
