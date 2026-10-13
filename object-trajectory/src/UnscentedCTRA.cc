#include "../include/UnscentedCTRA.h"

namespace ot {
namespace {

/// Wrap every angle row of a residual. Only theta is circular; the other five are not.
inline void wrapAngleRows(CtraState& residual)
{
    residual(kCtraTheta) = wrapPi(residual(kCtraTheta));
}

}  // namespace

Eigen::Matrix<double, kSigmaCount, 1> UnscentedCTRA::weightsMean() const
{
    const double lambda = sigma.lambda();
    const double denom = kCtraStateSize + lambda;

    Eigen::Matrix<double, kSigmaCount, 1> w;
    w.setConstant(1.0 / (2.0 * denom));
    w(0) = lambda / denom;
    return w;
}

Eigen::Matrix<double, kSigmaCount, 1> UnscentedCTRA::weightsCovariance() const
{
    Eigen::Matrix<double, kSigmaCount, 1> w = weightsMean();
    // The only difference from the mean weights, and the only place beta appears.
    w(0) += 1.0 - sigma.alpha * sigma.alpha + sigma.beta;
    return w;
}

/**
 * Place the sigma points.
 *
 * The layout matches FilterPy's MerweScaledSigmaPoints exactly, including which part of the
 * Cholesky factor is used, because the two are compared numerically.
 * FilterPy takes the ROWS of the UPPER factor U of (n + lambda) P.
 * Eigen's LLT gives the LOWER factor L with P = L L', so U = L' and row k of U is column k of L.
 * Hence `L.col(k)` below, and not `L.row(k)`.
 * Getting that backwards produces a filter that still runs and is quietly wrong, which is why it
 * is spelled out here and checked by the FilterPy comparison.
 */
bool UnscentedCTRA::sigmaPoints(SigmaPoints& out) const
{
    const double scale = kCtraStateSize + sigma.lambda();

    const Eigen::LLT<CtraCovariance> llt(scale * covariance);
    if (llt.info() != Eigen::Success) {
        return false;   // the covariance lost positive definiteness; the belief is unusable
    }
    const CtraCovariance lower = llt.matrixL();

    out.col(0) = state;
    for (int k = 0; k < kCtraStateSize; ++k) {
        out.col(k + 1) = state + lower.col(k);
        out.col(kCtraStateSize + k + 1) = state - lower.col(k);
    }
    return true;
}

/**
 * Weighted mean, with theta averaged circularly.
 *
 * A plain weighted sum of angles is wrong at the seam: averaging 179 degrees and -179 degrees
 * that way gives 0, when the answer is 180.
 * The fix is to average the WRAPPED OFFSETS from a reference and add the reference back, and the
 * natural reference is the centre point, which is already close to the answer.
 */
CtraState unscentedMean(const SigmaPoints& points,
                        const Eigen::Matrix<double, kSigmaCount, 1>& weights)
{
    CtraState mean = CtraState::Zero();

    // The five linear rows are a plain weighted sum.
    for (int i = 0; i < kSigmaCount; ++i) {
        mean += weights(i) * points.col(i);
    }

    // Theta is redone circularly, which overwrites the wrong value the loop above just put there.
    const double reference = points(kCtraTheta, 0);
    double offset = 0.0;
    for (int i = 0; i < kSigmaCount; ++i) {
        offset += weights(i) * wrapPi(points(kCtraTheta, i) - reference);
    }
    mean(kCtraTheta) = wrapPi(reference + offset);

    return mean;
}

bool UnscentedCTRA::predict(double dt)
{
    if (dt <= 0.0) {
        return true;   // nothing to do, and nothing broken
    }

    SigmaPoints points;
    if (!sigmaPoints(points)) {
        return false;
    }

    // Push every sigma point through the laboratory's CTRA model.
    // This is the whole reason there is no Jacobian anywhere in this file.
    for (int i = 0; i < kSigmaCount; ++i) {
        double px = points(kCtraPx, i);
        double py = points(kCtraPy, i);
        double v = points(kCtraSpeed, i);
        double theta = points(kCtraTheta, i);
        const double a = points(kCtraAccel, i);
        const double omega = points(kCtraYawRate, i);

        lab::ctra_predict(dt, px, py, v, a, theta, omega, false);

        points(kCtraPx, i) = px;
        points(kCtraPy, i) = py;
        points(kCtraSpeed, i) = v;
        points(kCtraTheta, i) = wrapPi(theta);
        // yaw rate and acceleration are constant under this model, so they carry over untouched
    }

    const Eigen::Matrix<double, kSigmaCount, 1> wm = weightsMean();
    const Eigen::Matrix<double, kSigmaCount, 1> wc = weightsCovariance();

    const CtraState mean = unscentedMean(points, wm);

    CtraCovariance spread = CtraCovariance::Zero();
    for (int i = 0; i < kSigmaCount; ++i) {
        CtraState residual = points.col(i) - mean;
        wrapAngleRows(residual);
        spread += wc(i) * (residual * residual.transpose());
    }

    // Process noise, added after the transform, as FilterPy does.
    spread(kCtraPx, kCtraPx) += qPos * dt;
    spread(kCtraPy, kCtraPy) += qPos * dt;
    spread(kCtraSpeed, kCtraSpeed) += qSpeed * dt;
    spread(kCtraTheta, kCtraTheta) += qTheta * dt;
    spread(kCtraYawRate, kCtraYawRate) += qYawRate * dt;
    spread(kCtraAccel, kCtraAccel) += qAccel * dt;

    state = mean;
    // Asymmetry accumulates in floating point and eventually costs the Cholesky its success.
    covariance = 0.5 * (spread + spread.transpose());
    return true;
}

Innovation UnscentedCTRA::innovationAgainst(const Measurement& z) const
{
    Innovation y;
    y.px = z.px - state(kCtraPx);
    y.py = z.py - state(kCtraPy);
    y.speed = z.speed - state(kCtraSpeed);
    // The report's heading is clockwise from north; the state's theta is not.
    // Compare them in the model's frame, and wrap.
    y.heading = wrapPi(lab::thetaFromHeading(z.heading) - state(kCtraTheta));
    return y;
}

InnovationCovariance UnscentedCTRA::innovationCovariance(const MeasurementNoise& r) const
{
    // Exact, not approximate: the observation selects the leading four states, so the unscented
    // transform of it reduces to this.
    InnovationCovariance s;
    for (int i = 0; i < kMeasuredSize; ++i) {
        for (int j = 0; j < kMeasuredSize; ++j) {
            s(i, j) = covariance(i, j);
        }
        s(i, i) += r[i];
    }
    return s;
}

bool UnscentedCTRA::update(const Measurement& z, const MeasurementNoise& r)
{
    using Innov = Eigen::Matrix<double, kMeasuredSize, 1>;
    using InnovCov = Eigen::Matrix<double, kMeasuredSize, kMeasuredSize>;
    using CrossCov = Eigen::Matrix<double, kCtraStateSize, kMeasuredSize>;

    const Innovation y = innovationAgainst(z);
    Innov residual;
    residual << y.px, y.py, y.speed, y.heading;

    // Pz = H P H' + R, and Pxz = P H'. Both exact for a selection observation.
    InnovCov pz = covariance.topLeftCorner<kMeasuredSize, kMeasuredSize>();
    for (int i = 0; i < kMeasuredSize; ++i) {
        pz(i, i) += r[i];
    }
    const CrossCov pxz = covariance.leftCols<kMeasuredSize>();

    // Solve rather than invert: Eigen's LDLT is both faster and better conditioned than forming
    // the inverse, and it tells us when the system is not solvable.
    const Eigen::LDLT<InnovCov> ldlt(pz);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    const CrossCov gain = ldlt.solve(pxz.transpose()).transpose();

    // Every state moves, the acceleration and yaw rate included, although the report mentioned
    // neither: the gain carries the correction across through the covariance's off-diagonal terms.
    state += gain * residual;
    state(kCtraTheta) = wrapPi(state(kCtraTheta));

    const CtraCovariance next = covariance - gain * pz * gain.transpose();
    covariance = 0.5 * (next + next.transpose());
    return true;
}

}  // namespace ot
