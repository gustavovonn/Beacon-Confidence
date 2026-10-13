/**
 * ukf_trace -- dump the filter's belief after every operation, for comparison against FilterPy.
 *
 * This exists only to be checked against a reference implementation.
 * scripts/validate_against_filterpy.py drives the same scripted sequence through FilterPy and
 * compares the two traces number by number.
 *
 * The input sequence is DETERMINISTIC and closed-form, with no random numbers anywhere.
 * That is deliberate: two random number generators in two languages will never agree, and a
 * comparison that depends on them agreeing tests the generators rather than the filters.
 *
 * Output format, one record per line, full precision:
 *
 *     <tag> <step> x0 x1 x2 x3 x4 x5 P00 P01 ... P55
 *
 * with tag either PREDICT or UPDATE, six state values and thirty-six covariance values in row
 * major order.
 */

#include "../include/UnscentedCTRA.h"

#include <cmath>
#include <cstdio>

using namespace ot;

namespace {

constexpr int kSteps = 200;
constexpr double kDt = 0.1;

/// The scripted measurement at step i, in the MODEL's frame, so that neither side has to agree
/// about heading conventions for this comparison to mean something.
Measurement scriptedMeasurement(int i)
{
    const double t = static_cast<double>(i);
    Measurement z;
    z.px = 0.3 * std::sin(0.10 * t);
    z.py = 2.0 * t + 0.2 * std::cos(0.07 * t);
    z.speed = 20.0 + 0.5 * std::sin(0.05 * t);
    // innovationAgainst converts the report's heading out of north-clockwise, so feed it the
    // heading whose theta is the value we want.
    z.heading = lab::headingFromTheta(0.4 + 0.01 * std::sin(0.03 * t));
    return z;
}

void emit(const char* tag, int step, const UnscentedCTRA& f)
{
    std::printf("%s %d", tag, step);
    for (int i = 0; i < kCtraStateSize; ++i) {
        std::printf(" %.17g", f.state(i));
    }
    for (int i = 0; i < kCtraStateSize; ++i) {
        for (int j = 0; j < kCtraStateSize; ++j) {
            std::printf(" %.17g", f.covariance(i, j));
        }
    }
    std::printf("\n");
}

}  // namespace

int main()
{
    UnscentedCTRA f;

    // A fixed, non-trivial starting belief. Deliberately not the identity, so that an error in
    // how the covariance is carried has somewhere to show.
    f.state << 0.0, 0.0, 18.0, 0.4, 0.05, 0.0;
    f.covariance = CtraCovariance::Zero();
    f.covariance(kCtraPx, kCtraPx) = 4.0;
    f.covariance(kCtraPy, kCtraPy) = 4.0;
    f.covariance(kCtraSpeed, kCtraSpeed) = 9.0;
    f.covariance(kCtraTheta, kCtraTheta) = 0.25;
    f.covariance(kCtraYawRate, kCtraYawRate) = 0.04;
    f.covariance(kCtraAccel, kCtraAccel) = 1.0;
    f.covariance(kCtraPx, kCtraSpeed) = 0.5;
    f.covariance(kCtraSpeed, kCtraPx) = 0.5;

    const MeasurementNoise r{1.0, 1.0, 0.5, 0.02};

    // The configuration the Python side must mirror exactly.
    std::printf("# CONFIG dt=%.17g steps=%d alpha=%.17g beta=%.17g kappa=%.17g\n",
                kDt, kSteps, f.sigma.alpha, f.sigma.beta, f.sigma.kappa);
    std::printf("# CONFIG q=%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
                f.qPos, f.qPos, f.qSpeed, f.qTheta, f.qYawRate, f.qAccel);
    std::printf("# CONFIG r=%.17g,%.17g,%.17g,%.17g\n", r.px, r.py, r.speed, r.heading);
    emit("INITIAL", -1, f);

    for (int i = 0; i < kSteps; ++i) {
        if (!f.predict(kDt)) {
            std::fprintf(stderr, "predict failed at step %d\n", i);
            return 1;
        }
        emit("PREDICT", i, f);

        if (!f.update(scriptedMeasurement(i), r)) {
            std::fprintf(stderr, "update failed at step %d\n", i);
            return 1;
        }
        emit("UPDATE", i, f);
    }
    return 0;
}
