#ifndef CHECKS_H_
#define CHECKS_H_

#include "MechanismConstants.h"
#include "EngineConfig.h"
#include "MotionModels.h"
#include "CoordinatePolicies.h"
#include "KinematicRegression.h"
#include <deque>
#include <cmath>
#include <algorithm>

struct PhysicalInvarianceResult {
    bool failed = false;
    double excessRatio = 0.0;
    double safeSpeed = 0.0;
    double dt = ConstantsPhysics::MIN_INTER_FRAME_TIME;
};

/**
 * @brief Evaluates physical invariance between two consecutive kinematic reports:
 * (1) Inter-frame transmission continuity (rejects duplicate receptions and stale gaps).
 * (2) Finite-difference speed plausibility (detects speed falsification and teleportation).
 * (3) Standstill noise floor / frozen position anomaly.
 *
 * When ablated (enabled == false) the verdict is cleared but the sanitized
 * safeSpeed and dt are still produced, since the dynamic threshold downstream
 * depends on them (paper Table III ablation).
 *
 * @tparam CoordPolicy Coordinate system policy (CartesianCoordinatePolicy or Wgs84CoordinatePolicy)
 */
template <typename CoordPolicy>
inline PhysicalInvarianceResult consistencyCheck(
    const KinematicState& pastState,
    const KinematicState& currentState,
    double speedToleranceMps = ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS,
    bool enabled = true)
{
    PhysicalInvarianceResult res;

    const double dt_raw = currentState.time - pastState.time;
    res.dt = (dt_raw < ConstantsPhysics::MIN_INTER_FRAME_TIME)
               ? ConstantsPhysics::MIN_INTER_FRAME_TIME
               : dt_raw;

    const double displacement = CoordPolicy::displacement(
        pastState.lat, pastState.lon,
        currentState.lat, currentState.lon);

    const double impliedSpeed = displacement / res.dt;

    const double speedTolerance = (speedToleranceMps > 0.0) ? speedToleranceMps 
                                                            : ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS;
    double speedDiscrepancy = 0.0;

    if (dt_raw >= ConstantsPhysics::MIN_INTER_FRAME_TIME && dt_raw <= ConstantsPhysics::MAX_INTER_FRAME_TIME) {
        const double avgReportedSpeed = 0.5 * (currentState.speed + pastState.speed);
        speedDiscrepancy = std::abs(avgReportedSpeed - impliedSpeed);
        const bool speedFalsified = (speedDiscrepancy > speedTolerance);

        // Frozen Position: node reports motion (> FROZEN_SPEED_THRESHOLD) but stayed motionless (< FROZEN_DISPLACEMENT_MIN)
        const bool frozenPosActive = (avgReportedSpeed > ConstantsPhysics::FROZEN_SPEED_THRESHOLD && 
                                      displacement < ConstantsPhysics::FROZEN_DISPLACEMENT_MIN);

        if (speedFalsified || frozenPosActive) {
            res.failed = true;
            res.excessRatio = (speedDiscrepancy > speedTolerance) ? 
                ((speedDiscrepancy - speedTolerance) / speedTolerance) : 1.0;
        }
    }

    // Sanitize speed used in dynamic threshold calculation so attacker cannot inflate bounds
    res.safeSpeed = std::min(currentState.speed, impliedSpeed + speedTolerance);

    if (!enabled) {
        res.failed = false;
        res.excessRatio = 0.0;
    }

    return res;
}

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
    const EngineConfig& params)
{
    return params.maxPosDev
         + params.contextSpeedWeight * (physics.safeSpeed * physics.dt)
         + params.contextYawWeight   * (std::abs(yawRate)  * physics.safeSpeed);
}


/**
 * @brief Rotational factor |omega| for the dynamic threshold (paper Eq. 10).
 */
inline double regressedYawRate(const std::deque<KinematicState>& history,
                               const KinematicState& pastState)
{
    double a_hat = pastState.accel;
    double w_hat = pastState.yawRate;
    KinematicRegression::fitOLS(history, pastState.accel, pastState.yawRate, a_hat, w_hat);
    return w_hat;
}

/**
 * @brief Predictive Check: trajectory prediction against the dynamic threshold.
 *
 * Predicts spatial position, computes distance error, and sets dynamic threshold.
 * If enableDynThresh == false, falls back to the static baseline maxPosDev.
 *
 * @return true if distError <= dynamicMaxPosDev (passed the predictive check).
 */
inline bool predictiveCheck(
    IMotionModel* drModel,
    const std::deque<KinematicState>& history,
    const KinematicState& pastState,
    const KinematicState& currentState,
    double dt,
    const EngineConfig& params,
    const PhysicalInvarianceResult& physics,
    double& outDistError,
    double& outDynamicMaxPosDev,
    bool enableDynThresh = true,
    bool enablePrediction = true)
{
    if (enablePrediction && drModel) {
        PredictedState expectedState;
        drModel->predict(history, pastState, expectedState, dt);
        outDistError = drModel->computeError(expectedState, currentState);
    } else {
        outDistError = 0.0;
    }

    if (enableDynThresh) {
        outDynamicMaxPosDev = computeDynamicThreshold(
            physics, regressedYawRate(history, pastState), params);
    } else {
        outDynamicMaxPosDev = params.maxPosDev;
    }

    return outDistError <= outDynamicMaxPosDev;
}

#endif // CHECKS_H_
