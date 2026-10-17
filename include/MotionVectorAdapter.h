#ifndef MOTION_VECTOR_ADAPTER_H_
#define MOTION_VECTOR_ADAPTER_H_

#include "ReceivedVector.h"
#include <cmath>
#include <map>
#include <vector>

/**
 * @brief Derivation helpers shared by the experiment trace loaders.
 *
 */
namespace MotionVectorAdapter {

/// Heading vector -> compass degrees in [0, 360).
inline double headingDegrees(double hx, double hy)
{
    double deg = std::atan2(hx, hy) * 180.0 / M_PI;
    if (deg < 0.0) deg += 360.0;
    return deg;
}

/**
 * @brief Fallback a and omega by two-point finite difference per sender.
 */
inline void applyFiniteDifference(std::vector<ReceivedVector>& vectors, double minInterval)
{
    struct PrevState { double time; double speed; double heading; };
    std::map<long, PrevState> prev;

    for (auto& rv : vectors) {
        auto it = prev.find(rv.sender);
        if (it != prev.end()) {
            double dt = rv.mv.time - it->second.time;
            if (dt < minInterval) dt = minInterval;

            rv.mv.accel = (rv.mv.speed - it->second.speed) / dt;

            double diff = rv.mv.heading - it->second.heading;
            diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
            rv.mv.yawRate = (diff * M_PI / 180.0) / dt;
        }
        prev[rv.sender] = {rv.mv.time, rv.mv.speed, rv.mv.heading};
    }
}

} // namespace MotionVectorAdapter

#endif // MOTION_VECTOR_ADAPTER_H_
