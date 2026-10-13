#ifndef OT_UNSCENTED_CTRA_H_
#define OT_UNSCENTED_CTRA_H_

/**
 * Unscented Kalman filter over the laboratory's CTRA motion model.
 *
 * WHY THIS EXISTS NEXT TO THE EKF
 *
 * The EKF needs the Jacobian of the motion model: twenty-five analytic derivatives, written by
 * hand, specific to our model, that no library will ever supply or check.
 * That is the one piece of this filter nobody else can validate for us.
 * A UKF needs no derivatives at all.
 * It picks a handful of representative states, pushes each through the REAL motion model, and
 * reads the mean and spread of what comes out.
 *
 * So this filter carries less that has to be taken on trust:
 *   - the motion model is the laboratory's own, taken verbatim (shared/LabCtraPredict.h);
 *   - the linear algebra is Eigen's (shared/eigen/);
 *   - the unscented transform matches FilterPy's formulation exactly, and is checked against it
 *     numerically by scripts/validate_against_filterpy.py.
 *
 * WHAT IS DELIBERATELY NOT UNSCENTED
 *
 * Only the prediction uses sigma points.
 * The measurement update does not, and that is not a shortcut: a motion vector reports the first
 * four states directly, so the observation is a plain selection and therefore LINEAR.
 * For a linear observation the unscented transform is exact and collapses to the ordinary Kalman
 * update -- `Pz` becomes `H P H' + R` and `Pxz` becomes `P H'`, identically, not approximately.
 * Running sigma points through it would cost more and compute the same numbers.
 * It also keeps gating cheap, which matters: the gate runs once per (track, vector) pair.
 *
 * THE STATE IS SIX WIDE, NOT FIVE
 *
 * The EKF carried [px, py, v, heading, yawRate] and no acceleration, which made its name wrong:
 * it was CTRV, constant turn rate and VELOCITY.
 * The laboratory's model is the real CTRA, so acceleration joins the state here.
 * Nothing measures it: a CAM carries acceleration and a CPM perceived object does not, so rather
 * than let the measurement change shape with the source, the filter infers acceleration from the
 * sequence of speeds.
 *
 * ANGLES
 *
 * `theta` lives in the state, so two places must respect that it is circular.
 * Averaging sigma points of an angle naively breaks at the seam, and so does subtracting two.
 * Both go through wrapping here, and the test suite checks the seam explicitly.
 *
 * CONVENTION WARNING
 *
 * This filter works in the laboratory model's frame, where theta is measured anticlockwise from
 * the +x axis.
 * A MotionVector carries a heading measured clockwise from north.
 * Convert only with `lab::thetaFromHeading` and `lab::headingFromTheta`, never by hand.
 */

#include "MeasurementSpace.h"
#include "../shared/LabCtraPredict.h"

#include <Eigen/Dense>

namespace ot {

// ---------------------------------------------------------------------------------------------
// Ordering
// ---------------------------------------------------------------------------------------------

/**
 * Where each quantity sits in the CTRA state, and therefore in its covariance.
 *
 * The first four deliberately line up with MeasuredIndex, which is what makes the observation a
 * plain selection of the leading rows and the update exactly linear.
 */
enum CtraIndex : int {
    kCtraPx = 0,        ///< [m] east
    kCtraPy = 1,        ///< [m] north
    kCtraSpeed = 2,     ///< [m/s]
    kCtraTheta = 3,     ///< [rad] ANTICLOCKWISE FROM +X, the laboratory model's convention
    kCtraYawRate = 4,   ///< [rad/s]
    kCtraAccel = 5,     ///< [m/s^2] never measured, always inferred
};
inline constexpr int kCtraStateSize = 6;

/// The six numbers the filter believes about one object.
using CtraState = Eigen::Matrix<double, kCtraStateSize, 1>;

/// How wrong those six may be, and which of those errors move together.
using CtraCovariance = Eigen::Matrix<double, kCtraStateSize, kCtraStateSize>;

/// One sigma point per column: the centre, then two per state dimension.
inline constexpr int kSigmaCount = 2 * kCtraStateSize + 1;
using SigmaPoints = Eigen::Matrix<double, kCtraStateSize, kSigmaCount>;

/**
 * The three numbers that decide where the sigma points go.
 *
 * These are van der Merwe's scaled unscented transform parameters.
 * alpha sets how far from the mean the points are placed.
 * beta carries prior knowledge of the distribution, and 2 is optimal for a Gaussian.
 * kappa is a secondary scaling, and 3 - n is the textbook choice for a Gaussian prior.
 *
 * WHY NOT alpha = 0.001, WHICH IS THE VALUE MOST EXAMPLES SHOW
 *
 * Because at n = 6 it is numerically unusable, and the failure is silent.
 * The weights divide by `n + lambda = alpha^2 (n + kappa)`, which for alpha = 0.001 is 6e-6, so
 * the weights come out at plus and minus a million and then cancel against each other.
 * The defining identity of the transform -- that the sigma points carry back exactly the mean
 * they were built from -- then holds only to about 1e-7 instead of to machine precision, and
 * every later number inherits that.
 * We measured it: 8.4e-08 of error in a quantity that should have none.
 *
 * The defaults below keep every weight at magnitude 1 or below:
 *
 *     alpha   kappa    n + lambda    largest |weight|
 *     0.001   0            6e-6          999999
 *     0.1     0            0.06              99
 *     0.5     0            1.5                3
 *     1.0     3 - n        3.0                1     <- chosen
 *
 * That choice is also the classical Julier-Uhlmann transform, so it is the conservative one in
 * both senses: best conditioned and most standard.
 */
struct SigmaParams {
    double alpha = 1.0;
    double beta = 2.0;
    double kappa = 3.0 - kCtraStateSize;

    /// lambda = alpha^2 (n + kappa) - n, the spread of the scaled transform.
    double lambda() const
    {
        return alpha * alpha * (kCtraStateSize + kappa) - kCtraStateSize;
    }
};

// ---------------------------------------------------------------------------------------------
// The filter
// ---------------------------------------------------------------------------------------------

struct UnscentedCTRA {
    CtraState state = CtraState::Zero();
    CtraCovariance covariance = CtraCovariance::Identity();

    SigmaParams sigma;

    /// Process noise per second, added after the unscented transform.
    /// FilterPy's UnscentedKalmanFilter adds Q the same way, which is what keeps the two
    /// comparable step for step.
    double qPos = 0.5;        ///< [m^2/s]
    double qSpeed = 1.0;      ///< [(m/s)^2/s]
    double qTheta = 0.05;     ///< [rad^2/s]
    double qYawRate = 0.05;   ///< [(rad/s)^2/s]
    double qAccel = 0.5;      ///< [(m/s^2)^2/s]

    /// Weights for the mean and for the covariance.
    /// They differ only in the first entry, which is where beta enters.
    Eigen::Matrix<double, kSigmaCount, 1> weightsMean() const;
    Eigen::Matrix<double, kSigmaCount, 1> weightsCovariance() const;

    /**
     * Place the sigma points around the current belief.
     *
     * Returns false if the covariance is not positive definite, which is the UKF's way of saying
     * the belief has become unusable.
     * The caller must not use the points in that case.
     */
    bool sigmaPoints(SigmaPoints& out) const;

    /// Move the belief forward by dt, through the laboratory's CTRA model.
    /// Returns false if the sigma points could not be placed.
    bool predict(double dt);

    /**
     * The gap between a report and the prediction, per measured quantity.
     *
     * The heading component is converted out of the model's frame before being compared, and the
     * difference is wrapped.
     */
    Innovation innovationAgainst(const Measurement& z) const;

    /// S = H P H' + R, exact because the observation is a selection of the leading four states.
    InnovationCovariance innovationCovariance(const MeasurementNoise& r) const;

    /// Fold a report in, with the ordinary Kalman update. Returns false if S is not invertible.
    bool update(const Measurement& z, const MeasurementNoise& r);

    /// Convenience: the heading in MotionVector convention, clockwise from north.
    double heading() const { return lab::headingFromTheta(state(kCtraTheta)); }
};

/**
 * Weighted mean of a set of sigma points, with the angle row averaged circularly.
 *
 * Exposed because the tests check it on its own, and because the circular average is the part
 * that is easy to get wrong and hard to notice.
 */
CtraState unscentedMean(const SigmaPoints& points,
                        const Eigen::Matrix<double, kSigmaCount, 1>& weights);

}  // namespace ot

#endif  // OT_UNSCENTED_CTRA_H_
