#ifndef MECHANISM_CONSTANTS_H
#define MECHANISM_CONSTANTS_H

#include <cmath>

// Parameters of the confidence-attribution mechanism evaluated in this work:
// the empirically calibrated weights, the ETSI-grounded interval bounds and the
// Beta confidence policy.

namespace ConstantsApp {

    constexpr size_t DEFAULT_HISTORY_WINDOW = 4;
}

namespace ConstantsPhysics {

    // -------------------------------------------------------------------------
    // Inter-frame Transmission Continuity Window
    // -------------------------------------------------------------------------

    // Minimum inter-frame time [s] below which packets are treated as duplicate
    // reception artifacts across multiple receiver logs.
    // Reference: ETSI EN 302 637-2 (minimum CAM generation interval is 100 ms (10 Hz);
    constexpr double MIN_INTER_FRAME_TIME = 0.1;

    // Maximum inter-frame time [s] for differential kinematic continuity.
    // Reference: adopted from F2MD, which gates its own differential consistency
    // J. Kamel, M. R. Ansari, J. Petit, A. Kaiser, I. B. Jemaa, and P. Urien,
    // "Simulation framework for misbehavior detection in vehicular networks,"
    constexpr double MAX_INTER_FRAME_TIME = 1.5;

    // -------------------------------------------------------------------------
    // Kinematic Plausibility and Standstill Noise Floor
    // -------------------------------------------------------------------------

    // Speed threshold [m/s] (~7.2 km/h) distinguishing active vehicular transit from
    // stationary GNSS random-walk receiver drift.
    // Reference: TODO
    constexpr double FROZEN_SPEED_THRESHOLD = 2.0;

    // Minimum spatial displacement [m] expected for a vehicle reporting v > 2.0 m/s.
    // At v = 2.0 m/s over dt = 0.2s, theoretical displacement is 0.4m. Any displacement < 0.3m
    // directly violates kinematic physical law (Frozen Position anomaly).
    // Reference: Physical invariance (d = v * dt) and GPS antenna positional jitter bound (~0.2m - 0.3m).
    constexpr double FROZEN_DISPLACEMENT_MIN = 0.3;

    // -------------------------------------------------------------------------
    // Operational Defaults (Parametrizable via EngineConfig)
    // -------------------------------------------------------------------------

    // Default kinematic speed discrepancy tolerance (|v_reported - v_implied|) [m/s].
    // Reference: VeReMi fault-free empirical distribution (bounds 99.5% of honest acceleration/braking maneuvers).
    constexpr double DEFAULT_SPEED_TOLERANCE_MPS = 4.0;

    // -------------------------------------------------------------------------
    // Context-Aware Dynamic Threshold (paper Eq. 10)
    //   d_max = d_max_0 + w_v * v_safe * dt + w_w * |w| * v_safe
    // -------------------------------------------------------------------------

    // Calibrated offline via quantile regression (q = 0.99) over honest senders
    // of ConstPos_1416 (optimize_weights.py; paper Sec. V-A).
    constexpr double DEFAULT_MAX_POS_DEV = 2.3027;          // d_max_0 [m]
    constexpr double DEFAULT_CONTEXT_SPEED_WEIGHT = 0.6203; // w_v (longitudinal displacement)
    constexpr double DEFAULT_CONTEXT_YAW_WEIGHT = 0.7519;   // w_w (rotational factor)

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
}


#endif
