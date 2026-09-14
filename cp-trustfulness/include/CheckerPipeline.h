#ifndef CHECKER_PIPELINE_H_
#define CHECKER_PIPELINE_H_

#include "ITrustEvaluator.h"
#include <deque>
#include <cmath>

namespace artery {

/**
 * @brief Computes the context-aware dynamic position deviation threshold.
 *
 * Applies offline-calibrated OLS regression weights to the sanitized kinematic
 * state, scaling the base intercept by speed-displacement and yaw-speed terms.
 *
 * Formula: d_max = d_max^0 + w_v*(safeSpeed*dt) + w_w*(|yawRate|*safeSpeed)
 *
 * Pure function with no side-effects. Callable by any consumer:
 * veremi_runner, artery integration, f2md integration.
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
 * If the motion vector failed Phase 1 (distError > dynamicMaxPosDev), this
 * check determines whether the deviation is attributable to a legitimate abrupt
 * maneuver (e.g. emergency braking, sharp lane change) rather than sensor fault.
 *
 * The centroid limit expands linearly with the time-span of the history window
 * and a tunable speed factor, giving the vehicle geometric freedom proportional
 * to how far it could reasonably have traveled from its recent trajectory centroid.
 *
 * Pure function with no side-effects. Callable by any consumer:
 * veremi_runner, artery integration, f2md integration.
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

} // namespace artery

#endif // CHECKER_PIPELINE_H_
