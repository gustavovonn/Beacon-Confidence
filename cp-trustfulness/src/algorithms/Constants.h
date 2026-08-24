#ifndef CONSTANTS_H
#define CONSTANTS_H


#include <cmath>
 
namespace ConstantsWGS84
{
    // -------------------------------------------------------------------------
    // WGS84 ellipsoid parameters
    // -------------------------------------------------------------------------
 
    // Semi-major axis (equatorial radius) [m]
    constexpr double WGS84_A = 6'378'137.0;
 
    // First eccentricity squared:  e² = 2f − f²
    constexpr double WGS84_E2 = 0.006'694'379'990'14;
 
    // -------------------------------------------------------------------------
    // Angular conversion
    // -------------------------------------------------------------------------
 
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;

    // -------------------------------------------------------------------------
    // Helper: WGS84 radii of curvature at a given geodetic latitude
    // -------------------------------------------------------------------------
 
    /**
     * @brief Meridional radius of curvature R_phi (north-south) [m].
     *
     * Used to convert a metric northward displacement dy into Δlatitude:
     *   Δφ = dy / R_phi
     *
     * Formula: R_phi = a(1 − e²) / W³,   W = sqrt(1 − e² sin²φ)
     *
     * @param latRad  Geodetic latitude in radians.
     */
    inline double meridionalRadius(double latRad)
    {
        const double sinLat = std::sin(latRad);
        const double W      = std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
        return WGS84_A * (1.0 - WGS84_E2) / (W * W * W);
    }
 
    /**
     * @brief Prime-vertical (normal) radius of curvature R_lambda (east-west) [m].
     *
     * Used to convert a metric eastward displacement dx into Δlongitude:
     *   Δλ = dx / (R_lambda · cos φ)
     *
     * Formula: R_lambda = a / W,   W = sqrt(1 − e² sin²φ)
     *
     * @param latRad  Geodetic latitude in radians.
     */
    inline double primeVerticalRadius(double latRad)
    {
        const double sinLat = std::sin(latRad);
        const double W      = std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
        return WGS84_A / W;
    }
}

// TODO: revisar
namespace ConstantsFixedLatLong
{
    /// Latitude conversion factor [m/degree] — paper eq. (4), (8).
    /// Approximately the meridional radius of curvature at mid-latitudes
    /// divided by (180/π), pre-converted to metres per degree.
    constexpr double R_PHI_DEG    = 110'574.0;
 
    /// Longitude conversion factor [m/degree] — paper eq. (5), (9).
    /// Equatorial value; scaled by cos(φ) at the call site.
    constexpr double R_LAMBDA_DEG = 111'320.0;

}

namespace ConstantsApp {

    // Yaw-rate magnitude below which the model falls back to CA [rad/s].
    // Chosen to avoid numerical blow-up in the 1/ω terms while remaining
    // physically meaningful (≈ 0.006 °/s).
    constexpr double YAW_RATE_THRESHOLD = 1e-4;

    /// Minimum OLS denominator magnitude below which the least-squares fit
    /// is considered degenerate and last-known state values are kept.
    constexpr double OLS_DENOM_THRESHOLD = 1e-6;
}

#endif