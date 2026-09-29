#ifndef F2MD_FAULTS_H_
#define F2MD_FAULTS_H_

/**
 * The position and speed malfunctions of VeReMi Extension [Kamel et al., IEEE ICC 2020, Sec. IV-C], with the
 * parameters of F2MD, the framework that generated the dataset (f2md/veins/.../F2MDParameters.h and
 * mdAttacks/MDAttack.cc). Paper 1 was evaluated on exactly these eight profiles.
 *
 *   const_pos           the position is one constant point of the playground
 *   const_pos_offset    a constant offset, U((1 - v) P, (1 + v) P) per axis (random sign, see below)
 *   random_pos          a random point of the playground in every message
 *   random_pos_offset   a random offset U(-P / 2, P / 2) per axis in every message
 *   const_speed         a constant velocity vector, U(0, S) per axis
 *   const_speed_offset  a constant velocity offset, U((1 - v) O, (1 + v) O) per axis
 *   random_speed        a random velocity vector U(0, S) per axis in every message
 *   random_speed_offset a random velocity offset of magnitude U((1 - v) O, (1 + v) O) and random sign per axis
 *
 * with v = parVar = 0.55, P = RandomPosOffset = 70 m, S = RandomSpeed = 40 m/s, O = RandomSpeedOffset = 7 m/s.
 * F2MD works in Veins playground coordinates (+x east, +y south); offsets keep that orientation here.
 *
 * One deviation: F2MD draws the constant position offset positive on both axes, so every faulty vehicle shifts into the same
 * quadrant by similar amounts. With CPMs this makes faulty authors confirm each other's false positions (an
 * accidental collusion that VeReMi, having no CPMs, never exposed). We keep F2MD's magnitudes and draw the sign
 * of each axis at random, as F2MD itself does for the random speed offset.
 */

#include "../../include/Geodesy.h"
#include <cmath>
#include <string>

namespace artery {
namespace F2md {

constexpr double kParVar = 0.55;
constexpr double kPosOffset = 70.0;    ///< [m]
constexpr double kSpeed = 40.0;        ///< [m/s]
constexpr double kSpeedOffset = 7.0;   ///< [m/s]

inline bool isPositionFault(const std::string& mode)
{
    return mode == "const_pos" || mode == "const_pos_offset" || mode == "random_pos" || mode == "random_pos_offset";
}

inline bool isSpeedFault(const std::string& mode)
{
    return mode == "const_speed" || mode == "const_speed_offset" || mode == "random_speed" ||
           mode == "random_speed_offset";
}

/// Playground (+x east, +y south) point to east/north.
inline LocalPoint fromPlayground(double x, double y)
{
    return LocalPoint{x, -y};
}

/// Draw of a constant or random offset per axis; `uniform(a, b)` is the caller's random stream.
template <typename Uniform>
LocalPoint constPosOffset(Uniform&& uniform)
{
    double x = uniform((1.0 - kParVar) * kPosOffset, (1.0 + kParVar) * kPosOffset);
    double y = uniform((1.0 - kParVar) * kPosOffset, (1.0 + kParVar) * kPosOffset);
    x = uniform(0.0, 1.0) < 0.5 ? x : -x;  // random sign (see above)
    y = uniform(0.0, 1.0) < 0.5 ? y : -y;
    return fromPlayground(x, y);
}

template <typename Uniform>
LocalPoint randomPosOffset(Uniform&& uniform)
{
    return fromPlayground(uniform(0.0, kPosOffset) - 0.5 * kPosOffset, uniform(0.0, kPosOffset) - 0.5 * kPosOffset);
}

template <typename Uniform>
LocalPoint randomVelocity(Uniform&& uniform)
{
    return fromPlayground(uniform(0.0, kSpeed), uniform(0.0, kSpeed));
}

template <typename Uniform>
LocalPoint constSpeedOffset(Uniform&& uniform)
{
    const double x = uniform((1.0 - kParVar) * kSpeedOffset, (1.0 + kParVar) * kSpeedOffset);
    const double y = uniform((1.0 - kParVar) * kSpeedOffset, (1.0 + kParVar) * kSpeedOffset);
    return fromPlayground(x, y);
}

template <typename Uniform>
LocalPoint randomSpeedOffset(Uniform&& uniform)
{
    double x = uniform((1.0 - kParVar) * kSpeedOffset, (1.0 + kParVar) * kSpeedOffset);
    double y = uniform((1.0 - kParVar) * kSpeedOffset, (1.0 + kParVar) * kSpeedOffset);
    x = uniform(0.0, 1.0) < 0.5 ? x : -x;
    y = uniform(0.0, 1.0) < 0.5 ? y : -y;
    return fromPlayground(x, y);
}

/// Velocity vector (east, north) of speed v along a heading clockwise from north.
inline LocalPoint velocity(double speed, double headingDeg)
{
    return Geodesy::direction(headingDeg, speed);
}

}  // namespace F2md
}  // namespace artery

#endif  // F2MD_FAULTS_H_
