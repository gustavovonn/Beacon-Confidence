#ifndef OT_MEASUREMENT_SPACE_H_
#define OT_MEASUREMENT_SPACE_H_

/**
 * What a motion vector reports, what disagreeing with it looks like, and how surprised that
 * entitles the filter to be.
 *
 * Nothing here knows which filter is downstream.
 * These four types and two functions are the whole vocabulary the association stage speaks, and
 * they survived the change from an extended to an unscented filter untouched, which is the point
 * of keeping them in a file of their own.
 */

#include <Eigen/Dense>
#include <cassert>
#include <cmath>

namespace ot {

/**
 * Where each quantity sits in a measurement.
 *
 * A motion vector reports position, speed and heading, and never the yaw rate or the
 * acceleration, so a measurement is four wide while the filter's state is six.
 * The ordering mirrors the leading four of the state's, which is what makes the observation a
 * plain selection and the measurement update exactly linear.
 */
enum MeasuredIndex : int {
    kMeasuredPx = 0,
    kMeasuredPy = 1,
    kMeasuredSpeed = 2,
    kMeasuredHeading = 3,
};
inline constexpr int kMeasuredSize = 4;

/// Wrap an angle to (-pi, pi]. The reason a heading residual is ever meaningful.
inline double wrapPi(double a)
{
    while (a > M_PI) { a -= 2.0 * M_PI; }
    while (a <= -M_PI) { a += 2.0 * M_PI; }
    return a;
}

/**
 * What one motion vector reports.
 *
 * A CAM and a CPM perceived object both reduce to this, which is the premise of the experiment:
 * past the parser the container is gone.
 */
struct Measurement {
    double px = 0.0;        ///< [m] east
    double py = 0.0;        ///< [m] north
    double speed = 0.0;     ///< [m/s]
    double heading = 0.0;   ///< [rad] clockwise from north, as CAM and CPM put it on the wire

    double& operator[](int i)
    {
        assert(i >= 0 && i < kMeasuredSize);
        switch (i) {
            case kMeasuredPx: return px;
            case kMeasuredPy: return py;
            case kMeasuredSpeed: return speed;
            default: return heading;
        }
    }
    double operator[](int i) const
    {
        assert(i >= 0 && i < kMeasuredSize);
        switch (i) {
            case kMeasuredPx: return px;
            case kMeasuredPy: return py;
            case kMeasuredSpeed: return speed;
            default: return heading;
        }
    }
};

/**
 * The gap between what a motion vector reported and what the filter predicted.
 *
 * Shaped like a Measurement and emphatically not one: a Measurement is a claim about the world,
 * an Innovation is a disagreement about it.
 * Everything the engine judges -- the gate, the normalised gap, the confidence -- is computed
 * from this and never from the raw report.
 * The heading component is always wrapped, and it is expressed in the filter's own angular frame
 * rather than the wire's.
 */
struct Innovation {
    double px = 0.0;
    double py = 0.0;
    double speed = 0.0;
    double heading = 0.0;

    double& operator[](int i)
    {
        assert(i >= 0 && i < kMeasuredSize);
        switch (i) {
            case kMeasuredPx: return px;
            case kMeasuredPy: return py;
            case kMeasuredSpeed: return speed;
            default: return heading;
        }
    }
    double operator[](int i) const
    {
        assert(i >= 0 && i < kMeasuredSize);
        switch (i) {
            case kMeasuredPx: return px;
            case kMeasuredPy: return py;
            case kMeasuredSpeed: return speed;
            default: return heading;
        }
    }
};

/**
 * How much the filter distrusts one report, per measured quantity, as a variance.
 *
 * Only the diagonal: the four quantities a motion vector carries are treated as independently
 * noisy.
 * Must stay greater than zero even on a perfect dataset, because it stands for model mismatch and
 * not only for sensor error.
 * At zero the covariance collapses, every gate shuts, and nothing ever associates again, which
 * the parameter sweep has observed rather than merely predicted.
 */
struct MeasurementNoise {
    double px = 1.0;          ///< [m^2]
    double py = 1.0;          ///< [m^2]
    double speed = 0.5;       ///< [(m/s)^2]
    double heading = 0.02;    ///< [rad^2]

    double operator[](int i) const
    {
        assert(i >= 0 && i < kMeasuredSize);
        switch (i) {
            case kMeasuredPx: return px;
            case kMeasuredPy: return py;
            case kMeasuredSpeed: return speed;
            default: return heading;
        }
    }
};

/**
 * Covariance in measurement space: the innovation covariance S = H P H' + R.
 *
 * How surprised the filter is entitled to be, combining its own doubt with its distrust of the
 * report.
 * Dividing an Innovation by it turns a gap in metres into a gap in units of justified surprise.
 */
using InnovationCovariance = Eigen::Matrix<double, kMeasuredSize, kMeasuredSize>;

/**
 * Squared Mahalanobis distance y' S^-1 y.
 *
 * The gap measured in units of how surprised we are entitled to be, which is the only scale on
 * which a gap means anything: two metres is nothing when the filter is lost and a great deal when
 * it is confident.
 * Returns a negative value if S cannot be decomposed, which the caller must treat as "no
 * candidate" rather than as a small distance.
 *
 * Solved through Eigen's LLT rather than by forming the inverse: better conditioned, faster, and
 * it reports failure instead of returning nonsense.
 *
 * LLT and not LDLT, which was the first attempt and was wrong.
 * LDLT succeeds on a merely semi-definite matrix and then returns zeros where it should return
 * infinities, so a degenerate covariance came back as distance ZERO -- a perfect match -- and
 * every report would have sailed through the gate.
 * LLT succeeds only for a positive definite matrix, which is exactly the condition under which
 * this distance means anything.
 * (Caught by tests/test_measurement_space.cc, which fed it a covariance of all zeros.)
 */
inline double mahalanobis(const Innovation& y, const InnovationCovariance& s)
{
    Eigen::Matrix<double, kMeasuredSize, 1> residual;
    residual << y.px, y.py, y.speed, y.heading;

    const Eigen::LLT<InnovationCovariance> llt(s);
    if (llt.info() != Eigen::Success) {
        return -1.0;
    }
    const Eigen::Matrix<double, kMeasuredSize, 1> solved = llt.solve(residual);
    if (!solved.allFinite()) {
        return -1.0;
    }
    return residual.dot(solved);
}

}  // namespace ot

#endif  // OT_MEASUREMENT_SPACE_H_
