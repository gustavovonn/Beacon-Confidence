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

namespace ConstantsApp {

    // Yaw-rate magnitude below which the model falls back to CA [rad/s].
    // Chosen to avoid numerical blow-up in the 1/ω terms while remaining
    // physically meaningful (≈ 0.006 °/s).
    constexpr double YAW_RATE_THRESHOLD = 1e-4;

    /// Minimum OLS denominator magnitude below which the least-squares fit
    /// is considered degenerate and last-known state values are kept.
    constexpr double OLS_DENOM_THRESHOLD = 1e-6;
}

namespace ConstantsPhysics {

    // -------------------------------------------------------------------------
    // Inter-frame Transmission Continuity Window
    // -------------------------------------------------------------------------

    // Minimum inter-frame time [s] below which packets are treated as duplicate
    // reception artifacts across multiple receiver logs.
    // Reference: ETSI EN 302 637-2 (minimum CAM generation interval is 100 ms / 10 Hz;
    // in 1 Hz VeReMi, duplicate logs of the same broadcast occur at dt < 0.001s).
    constexpr double MIN_INTER_FRAME_TIME = 0.2;

    // Maximum inter-frame time [s] for differential kinematic continuity.
    // Beyond 2.0s (>2 dropped beacons at 1 Hz), finite-difference velocity estimation
    // is invalidated by unobserved driver maneuvers.
    // Reference: Grounded in F2MD MAX_TIME_DELTA (1.5s - 2.0s, Wiedersheim et al. 2012).
    constexpr double MAX_INTER_FRAME_TIME = 2.0;

    // -------------------------------------------------------------------------
    // Kinematic Plausibility and Standstill Noise Floor
    // -------------------------------------------------------------------------

    // Speed threshold [m/s] (~7.2 km/h) distinguishing active vehicular transit from
    // stationary GNSS random-walk receiver drift.
    // Reference: Low-cost automotive GNSS receiver noise floor (random walk velocity jitter up to 1.5 m/s).
    constexpr double FROZEN_SPEED_THRESHOLD = 2.0;

    // Minimum spatial displacement [m] expected over dt >= 0.2s for a vehicle reporting v > 2.0 m/s.
    // At v = 2.0 m/s over dt = 0.2s, theoretical displacement is 0.4m. Any displacement < 0.3m
    // directly violates kinematic physical law (Frozen Position anomaly).
    // Reference: Physical invariance (d = v * dt) and GPS antenna positional jitter bound (~0.2m - 0.3m).
    constexpr double FROZEN_DISPLACEMENT_MIN = 0.3;

    // -------------------------------------------------------------------------
    // Operational Defaults (Parametrizable via TrustEvaluationParams)
    // -------------------------------------------------------------------------

    // Default kinematic speed discrepancy tolerance (|v_reported - v_implied|) [m/s].
    // Reference: VeReMi dens_00 empirical distribution (bounds 99.5% of honest acceleration/braking maneuvers).
    constexpr double DEFAULT_SPEED_TOLERANCE_MPS = 4.0;

    // Default Beta reputation filter inertia (effective memory horizon N_eff).
    // Reference: Jøsang (2002) exponential aging with λ = 0.90 (N_eff = 1 / (1 - λ) = 10).
    constexpr double DEFAULT_PRIOR_WEIGHT = 10.0;

    // Default Reward Policy (Bayesian evidence increments)
    constexpr double DEFAULT_ALPHA_REWARD = 1.0;          // Full confirmation increment
    constexpr double DEFAULT_ALPHA_MANEUVER_REWARD = 0.5; // Maneuver confirmation increment

    // Default Penalty Policy (Severity-graduated β increments)
    // Reference: Derived from ETSI TS 103 759 reaction latency (K=2 frames under N=10)
    constexpr double DEFAULT_SOFT_MARGIN_RATIO = 1.35;    // 35% buffer for GPS jitter
    constexpr double DEFAULT_PENALTY_SOFT_BASE = 0.10;    // Jitter base penalty
    constexpr double DEFAULT_PENALTY_SOFT_SCALE = 0.50;   // Jitter multiplier
    constexpr double DEFAULT_PENALTY_SEVERE_BASE = 5.0;   // Severe violation base (guarantees 2-frame eviction)
    constexpr double DEFAULT_PENALTY_SEVERE_SCALE = 5.0;  // Severe violation multiplier
    constexpr double DEFAULT_MAX_BETA_PENALTY = 15.0;     // Single-frame penalty ceiling
    constexpr double DEFAULT_CENTROID_SPEED_FACTOR = 0.75;// Geometric expansion factor
}

#endif