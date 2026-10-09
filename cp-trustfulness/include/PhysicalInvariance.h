#ifndef PHYSICAL_INVARIANCE_H_
#define PHYSICAL_INVARIANCE_H_

#include "IDeadReckoningModel.h"
#include "Constants.h"
#include "CoordinatePolicies.h"
#include <cmath>
#include <algorithm>

namespace artery {

struct PhysicalInvarianceResult {
    bool failed = false;
    double excessRatio = 0.0;
    double safeSpeed = 0.0;
    double dt = 0.1;
};

/**
 * @brief Evaluates physical invariance between two consecutive kinematic reports:
 * (1) Inter-frame transmission continuity (rejects duplicate receptions and stale gaps).
 * (2) Finite-difference speed plausibility (detects speed falsification and teleportation).
 * (3) Standstill noise floor / frozen position anomaly.
 *
 * @tparam CoordPolicy Coordinate system policy (CartesianCoordinatePolicy or Wgs84CoordinatePolicy)
 */
template <typename CoordPolicy>
inline PhysicalInvarianceResult checkPhysicalInvariants(
    const KinematicState& pastState,
    const KinematicState& currentState,
    double speedToleranceMps = ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS,
    double minInterFrameTime = ConstantsPhysics::MIN_INTER_FRAME_TIME)
{
    PhysicalInvarianceResult res;

    const double dt_raw = currentState.time - pastState.time;
    res.dt = (dt_raw <= 0.0) ? 0.1 : dt_raw;

    const double displacement = CoordPolicy::displacement(
        pastState.lat, pastState.lon,
        currentState.lat, currentState.lon);

    const double impliedSpeed = displacement / res.dt;

    const double speedTolerance = (speedToleranceMps > 0.0) ? speedToleranceMps 
                                                            : ConstantsPhysics::DEFAULT_SPEED_TOLERANCE_MPS;
    double speedDiscrepancy = 0.0;

    // Only evaluate inter-frame physical invariance on valid consecutive transmissions:
    // (1) dt_raw >= minInterFrameTime: a duplicate guard, NOT a physics constant. The 0.2 s default exists
    //     because the VeReMi dataset repeats one message across several receiver logs, so Paper 1's ingestion
    //     had to skip repeats. In a simulator each receiver sees each message once, and a 0.2 s floor instead
    //     suppresses phase 0 exactly when a vehicle manoeuvres and its CAM rate rises to 10 Hz. Callers that
    //     do not read a VeReMi trace should pass their own duplicate threshold.
    // (2) dt_raw <= MAX_INTER_FRAME_TIME: ignores large communication gaps where kinematics are discontinuous (F2MD MAX_TIME_DELTA)
    if (dt_raw >= minInterFrameTime && dt_raw <= ConstantsPhysics::MAX_INTER_FRAME_TIME) {
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

    return res;
}

} // namespace artery

#endif // PHYSICAL_INVARIANCE_H_
