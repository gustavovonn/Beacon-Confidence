#ifndef OT_SHARED_LAB_CTRA_PREDICT_H_
#define OT_SHARED_LAB_CTRA_PREDICT_H_

// =============================================================================================
// PROVENANCE
//
// Taken from the laboratory's own code, not written here.
//
//   source   sdav/smartdata/include/adas/aeb/aeb.h, lines 196-258
//   symbol   AEB::ctra_predict
//   taken    2026-10-10
//   purpose  there it predicts a vehicle's path for autonomous emergency braking;
//            here it is the motion model of the object-trajectory filter
//
// The expressions below are character-identical to the original except where the CHANGES section
// says otherwise.
// Every change is listed.
// If you touch this file, add to that list.
//
// ---------------------------------------------------------------------------------------------
// CHANGES, each one and why
//
//   1. float -> double, and fabsf/cosf/sinf -> std::fabs/std::cos/std::sin.
//      The rest of this experiment carries state in double, and a filter that mixes the two loses
//      precision in its covariance for no gain.
//      Purely a widening: every value the original could represent, this one represents exactly.
//      tests/test_lab_ctra.cc checks the two agree to float precision over a sweep of inputs.
//
//   2. The guard `if (v < 1 and a < 1e-1) { return; }` is REMOVED.
//      In the original it skips the prediction for a nearly-stopped vehicle, which is reasonable
//      for a braking system that has nothing to decide in that case.
//      In a filter it is not: a track of a stopped car would never advance, its covariance would
//      never grow, and its gate would never widen, so a car pulling away from a traffic light
//      would fall outside its own gate and start a new trajectory.
//      LuST is urban and full of stopped vehicles, so this would have been a frequent failure.
//
//   3. One line added: `(void) ego;`.
//      The parameter is used only inside the original's commented-out debug output, so the
//      compiler warns that it is unused.
//      The parameter is kept rather than deleted so the signature still matches the original.
//
//   4. Nothing else.
//      The commented-out `db<SmartData>(INF)` block is the original's, left exactly where it was,
//      so that a diff against aeb.h shows this file is otherwise untouched.
//
// ---------------------------------------------------------------------------------------------
// CONVENTIONS THIS MODEL USES, which are NOT the ones the rest of the experiment uses
//
// Heading: theta is measured anticlockwise from the +x axis, so the vehicle moves by
// `cos(theta)` in x and `sin(theta)` in y.
// A MotionVector instead carries a heading measured CLOCKWISE FROM NORTH, which is what CAM and
// CPM put on the wire.
// The two are converted by `thetaFromHeading` / `headingFromTheta` below, and nothing else in
// this experiment may convert them by hand.
//
// Straight-line threshold: the original treats |omega| < 2e-2 rad/s as straight.
// That is loose -- at 20 m/s it calls a 1000 m turning radius a straight line -- but it is the
// laboratory's choice and it is left alone.
// It costs accuracy only in very gentle curves, and the filter's process noise absorbs that.
// =============================================================================================

#include <cmath>

namespace ot {
namespace lab {

/// Convert a MotionVector heading (radians clockwise from north) to this model's theta
/// (radians anticlockwise from the +x axis).
inline double thetaFromHeading(double headingRad)
{
    return M_PI / 2.0 - headingRad;
}

/// Convert this model's theta back to a MotionVector heading.
/// Exact inverse of thetaFromHeading, which tests/test_lab_ctra.cc checks round-trips.
inline double headingFromTheta(double thetaRad)
{
    return M_PI / 2.0 - thetaRad;
}

// ---------------------------------------------------------------------------------------------
// BEGIN borrowed code -- see PROVENANCE above
// ---------------------------------------------------------------------------------------------

// CTRA (Constant Turn Rate and Acceleration) motion model.
// Predicts future position (px, py) of a vehicle after dt seconds,
// given its current kinematic state: position, speed, heading,
// yaw rate (omega), and longitudinal acceleration.
//
// Two cases:
//   1. |omega| < epsilon: straight-line motion (avoids division by zero)
//   2. otherwise: curvilinear arc using full CTRA equations
inline void ctra_predict(double dt, double &px, double &py, double &v, const double a,
                         double &theta, const double omega, bool ego)
{
    (void) ego;   // CHANGE 3: see PROVENANCE

    // Speed and heading at the end of interval
    double v_dt = v + a * dt;
    double theta_dt = theta + omega * dt;

    // CHANGE 2: the original's `if (v < 1 and a < 1e-1) { return; }` stood here.

    // db<SmartData>(INF) << (ego ? "EGO" : "GVT") << ", "
    //                     << px << ", "
    //                     << py << ", "
    //                     << v << ", "
    //                     << a << ", "
    //                     << theta << ", "
    //                     << omega << ", "
    //                     << endl;

    if (std::fabs(omega) < 2e-2)
    {
        // Straight-line: integrate position along constant heading
        // dist = v*t + 0.5*a*t²
        double dist = v * dt + 0.5 * a * dt * dt;
        px = px + dist * std::cos(theta);
        py = py + dist * std::sin(theta);
    }
    else
    {
        // pre computing
        double div_omega = (1.0 / omega);
        double a_over_squared_omega = (a / (omega * omega));

        // Curvilinear CTRA:
        // The vehicle follows an arc whose radius changes as speed changes.
        // Position is derived by integrating the heading angle over time,
        // where heading evolves linearly: theta(t) = theta + omega*t
        //
        // X: (1/omega) * (v(t)*sin(theta(t)) - v0*sin(theta0))
        //  + (a/omega²) * (cos(theta(t)) - cos(theta0))
        //
        // Y: -(1/omega) * (v(t)*cos(theta(t)) - v0*cos(theta0))
        //  + (a/omega²) * (sin(theta(t)) - sin(theta0))
        px = px + div_omega * (v_dt * std::sin(theta_dt) - v * std::sin(theta)) + a_over_squared_omega * (std::cos(theta_dt) - std::cos(theta));

        py = py - div_omega * (v_dt * std::cos(theta_dt) - v * std::cos(theta)) + a_over_squared_omega * (std::sin(theta_dt) - std::sin(theta));
    }

    // Propagating to the next iteration
    v = v_dt;
    theta = theta_dt;
}

// ---------------------------------------------------------------------------------------------
// END borrowed code
// ---------------------------------------------------------------------------------------------

}  // namespace lab
}  // namespace ot

#endif  // OT_SHARED_LAB_CTRA_PREDICT_H_
