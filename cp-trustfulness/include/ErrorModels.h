#ifndef ERROR_MODELS_H_
#define ERROR_MODELS_H_

/**
 * Error models of the simulation, each taken from a cited source. The same values parameterise the receiver's
 * gates, so the receiver assumes exactly the sensor and positioning quality the simulation produces.
 *
 *   GNSS   the error model published with VeReMi Extension [Kamel et al., "VeReMi Extension: A Dataset for
 *          Comparable Evaluation of Misbehavior Detection in VANETs", IEEE ICC 2020, Sec. IV-B], as printed. The
 *          dataset's own logged noise follows F2MD's implementation and differs in detail.
 *   Radar  measurement noise: Jacobs et al., "On the Accuracy of Automotive Radar Tracking", IEEE VTC2023-Spring;
 *          field of view: 100 m and +-25 deg, bracketed by the configurations of the collective-perception
 *          study itself [ETSI TR 103 562 V2.1.1, cl. 5.3.3.2, Table 4: 65 m/+-40 deg with 150 m/+-5 deg, or
 *          150 m/360 deg] -- a better source than ITU-R M.2057-1 (2018) Radar B, which is a spectrum-sharing
 *          recommendation; detection and line of sight: Artery's field-of-view sensor. Richer perception error models exist (e.g. CoPEM, Piazzoni et al., ITSC 2022);
 *          this is the simplest cited one.
 */

#include "Geodesy.h"
#include <algorithm>
#include <cmath>

namespace artery {

/**
 * VeReMi Extension positioning errors, per vehicle:
 *   position  E0 ~ U(-b, b) per axis, then E_t ~ N((E0 + E_{t-1}) / 2, (c * E0)^2)       b = 5 m, c = 0.03
 *   heading   E_H = E0H * exp(-k * v), E0H ~ U(-20, 20) deg                                k = 0.1 s/m
 *   speed     V_E = V * (1 + E0V), E0V ~ N(0, s^2)                                         s = 0.00016
 * The error of a vehicle is almost a constant offset (up to 5 m per axis) with a small jitter around it.
 */
struct GnssErrorModel {
    double positionBound = 5.0;      ///< b [m]
    double jitterFraction = 0.03;    ///< c
    double headingBoundDeg = 20.0;   ///< [deg]
    double headingSpeedDecay = 0.1;  ///< k [s/m]
    double speedSigma = 0.00016;     ///< s (relative)

    /// Standard deviation of the position error per axis across vehicles: that of U(-b, b).
    double positionSigma() const { return positionBound / std::sqrt(3.0); }

    /// Standard deviation of the heading error at speed v [rad].
    double headingSigma(double speed) const
    {
        return headingBoundDeg / std::sqrt(3.0) * ConstantsWGS84::DEG_TO_RAD * std::exp(-headingSpeedDecay * speed);
    }

    /// Heading error [deg] of a vehicle whose drawn E0H is `e0Deg`, at speed v.
    double headingError(double e0Deg, double speed) const { return e0Deg * std::exp(-headingSpeedDecay * speed); }

    /// Next position error along one axis. `normal(mean, sigma)` draws from the caller's random stream.
    template <typename Normal>
    double step(double e0, double previous, Normal&& normal) const
    {
        const double sigma = jitterFraction * std::abs(e0);
        const double mean = 0.5 * (e0 + previous);
        return sigma > 0.0 ? normal(mean, sigma) : mean;
    }
};

/**
 * Radar measurement model. A detection is measured in polar coordinates from the sensor mount with Gaussian
 * range and azimuth noise (Jacobs et al.: sigma_r = 0.4 m, sigma_theta = 0.052 rad = 3 deg, sigma_d = 0.2 m/s).
 * Those three values are ASSUMED parameters of a theoretical analysis, not a hardware measurement, and the
 * study's target is vulnerable road users rather than vehicles; sigma_theta = 3 deg is conservative for a
 * production sensor. Whether an object is detected at all is Artery's field-of-view sensor with line of
 * sight, over 100 m and +-25 deg (see above).
 *
 * The receiver never counts a MISSING detection against a claim. A non-detection cannot contradict a
 * detection: a station that does not report an object may not be looking there, may lack line of sight, or
 * may not be reporting at all. Absence of support is therefore missing evidence, never negative evidence,
 * which is why corroboration is positive-only and why ghost objects are out of scope by principle.
 */
struct RadarErrorModel {
    double rangeSigma = 0.4;       ///< [m]
    double azimuthSigmaDeg = 3.0;  ///< [deg]
    double speedSigma = 0.2;       ///< [m/s]

    /**
     * Position variance per axis of one detection at `range` [m^2].
     *
     * A polar measurement with independent range and azimuth noise has an error ELLIPSE, elongated across
     * range: at 100 m the cross-range standard deviation is 5.24 m against 0.40 m in range, a factor of 13.
     * We do not model that ellipse, we BOUND it: taking the larger of the two semi-axes makes the circular
     * gate of radius kappa * sqrt(variance) contain the true error region at every bearing.
     *
     * The alternative, averaging the two variances, preserves the expected squared error but yields a circle
     * SMALLER than the long semi-axis, so it under-covers across range and manufactures false positives for
     * objects seen broadside at long range. Over-covering is the only safe direction for a gate, and a bound
     * needs no appeal to tracking theory to justify. Measured cost of the change: gates widen 33-41 %.
     */
    double positionVariance(double range) const
    {
        const double crossRange = range * azimuthSigmaDeg * ConstantsWGS84::DEG_TO_RAD;
        return std::max(rangeSigma * rangeSigma, crossRange * crossRange);
    }

    /// Measures `target` from `mount` (local tangent plane, metres). `normal(mean, sigma)` draws the noise.
    template <typename Normal>
    LocalPoint measure(const LocalPoint& mount, const LocalPoint& target, Normal&& normal) const
    {
        const LocalPoint d{target.east - mount.east, target.north - mount.north};
        const double range = std::max(0.0, Geodesy::norm(d) + normal(0.0, rangeSigma));
        const double bearing = Geodesy::bearing(d) + normal(0.0, azimuthSigmaDeg);
        const LocalPoint m = Geodesy::direction(bearing, range);
        return LocalPoint{mount.east + m.east, mount.north + m.north};
    }
};

}  // namespace artery

#endif  // ERROR_MODELS_H_
