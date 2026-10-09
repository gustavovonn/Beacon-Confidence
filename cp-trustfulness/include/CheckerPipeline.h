#ifndef CHECKER_PIPELINE_H_
#define CHECKER_PIPELINE_H_

#include "ITrustEvaluator.h"
#include "PhysicalInvariance.h"
#include "IDeadReckoningModel.h"
#include <deque>
#include <cmath>

namespace artery {

/**
 * @brief Ablation configuration flags to systematically disable individual
 * pipeline phases and features.
 */
struct AblationConfig {
    bool enablePhase0    = true; // Phase 0: Hard physical invariance checks
    bool enableDynThresh = true; // Phase 1: Context-aware dynamic threshold (false -> static maxPosDev)
    bool enablePhase2    = false; // Phase 2: Centroid plausibility maneuver rescue (disabled - zero empirical impact)
    bool enableOLS       = true; // DR smoothing (false -> bare CTRA / lastState only)
    bool enableDR        = true; // Phase 1 Dead-Reckoning (false -> no spatial prediction, distError = 0)
};

/**
 * @brief Computes the context-aware dynamic position deviation threshold.
 *
 * Formula: d_max = d_max^0 + w_v*(safeSpeed*dt) + w_w*(|yawRate|*safeSpeed)
 *
 * @param physics    Pre-computed physical invariance result (carries safeSpeed and dt).
 * @param yawRate    Reported yaw rate of the past kinematic state [rad/s].
 * @param params     Trust evaluation parameters (maxPosDev, contextSpeedWeight, contextYawWeight).
 * @return           Dynamic maximum position deviation threshold [m].
 */
inline double computeDynamicThreshold(
    const PhysicalInvarianceResult& physics,
    double yawRate,
    const TrustEvaluationParams& params)
{
    return params.maxPosDev
         + params.contextSpeedWeight * (physics.safeSpeed * physics.dt)
         + params.contextYawWeight   * (std::abs(yawRate)  * physics.safeSpeed);
}

/**
 * @brief Evaluates the Phase 2 centroid deviation check.
 *
 * @param history             Sliding window of past kinematic states for this sender.
 * @param currentState        Current (incoming) kinematic state.
 * @param centroidDeviation   Pre-computed distance from received position to window centroid [m].
 * @param dynamicMaxPosDev    Pre-computed dynamic threshold from computeDynamicThreshold() [m].
 * @param centroidSpeedFactor Tunable expansion coefficient (default 0.75, ETSI-grounded).
 * @return true if the deviation is within the centroid limit (motion vector forgiven as maneuver).
 */
inline bool computeCentroidCheck(
    const std::deque<KinematicState>& history,
    const KinematicState& currentState,
    double centroidDeviation,
    double dynamicMaxPosDev,
    double centroidSpeedFactor)
{
    if (history.empty()) return false;
    const double timeWindow    = currentState.time - history.front().time;
    const double centroidLimit = (currentState.speed * timeWindow * centroidSpeedFactor)
                               + dynamicMaxPosDev;
    return centroidDeviation <= centroidLimit;
}

// =========================================================================
// Standardized Phase Methods (1-to-1 Paper Alignment & Experiment Agnostic)
// =========================================================================

/**
 * @brief Phase 0: Hard Physical Invariance Verification.
 *
 * Checks continuity, velocity bounds, and freeze anomalies.
 * If ablated (enabled == false), marks failed as false while preserving
 * sanitized safeSpeed and dt for downstream calculations.
 */
template <typename CoordPolicy>
inline PhysicalInvarianceResult phase0_PhysicalInvariance(
    const KinematicState& pastState,
    const KinematicState& currentState,
    double speedToleranceMps = ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS,
    bool enabled = true,
    double minInterFrameTime = ConstantsPhysics::MIN_INTER_FRAME_TIME)
{
    PhysicalInvarianceResult result = checkPhysicalInvariants<CoordPolicy>(
        pastState, currentState, speedToleranceMps, minInterFrameTime);

    if (!enabled) {
        result.failed = false;
        result.excessRatio = 0.0;
    }
    return result;
}

/**
 * @brief Phase 1: Kinematic Consistency via Dead-Reckoning & Dynamic Threshold.
 *
 * Predicts spatial position, computes distance error, and sets dynamic threshold.
 * If enableDynThresh == false, falls back to the static baseline maxPosDev.
 *
 * @return true if distError <= dynamicMaxPosDev (passed Phase 1 consistency).
 */
inline bool phase1_KinematicConsistency(
    IDeadReckoningModel* drModel,
    const std::deque<KinematicState>& history,
    const KinematicState& pastState,
    const KinematicState& currentState,
    double dt,
    const TrustEvaluationParams& params,
    const PhysicalInvarianceResult& physics,
    double& outDistError,
    double& outDynamicMaxPosDev,
    bool enableDynThresh = true,
    bool enableDR = true)
{
    if (enableDR && drModel) {
        PredictedState expectedState;
        drModel->predict(history, pastState, expectedState, dt);
        outDistError = drModel->computeError(expectedState, currentState);
    } else {
        outDistError = 0.0;
    }

    if (enableDynThresh) {
        outDynamicMaxPosDev = computeDynamicThreshold(physics, pastState.yawRate, params);
    } else {
        outDynamicMaxPosDev = params.maxPosDev;
    }

    return outDistError <= outDynamicMaxPosDev;
}

/**
 * @brief Phase 2: Centroid Plausibility / Maneuver Rescue.
 *
 * Evaluates whether a deviation exceeding Phase 1 threshold is forgiven
 * as an abrupt maneuver within the expanding centroid hull.
 * If ablated (enabled == false), returns false.
 */
template <typename CoordPolicy>
inline bool phase2_CentroidPlausibility(
    const std::deque<KinematicState>& history,
    const KinematicState& currentState,
    double dynamicMaxPosDev,
    double centroidSpeedFactor,
    double& outCentroidDeviation,
    bool enabled = true)
{
    outCentroidDeviation = CoordPolicy::centroidDeviation(
        history, currentState.lat, currentState.lon);

    if (!enabled || history.empty()) {
        return false;
    }

    return computeCentroidCheck(
        history, currentState, outCentroidDeviation, dynamicMaxPosDev, centroidSpeedFactor);
}

} // namespace artery

#endif // CHECKER_PIPELINE_H_
