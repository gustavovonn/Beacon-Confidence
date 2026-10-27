#ifndef BEACON_CONFIDENCE_ENGINE_H_
#define BEACON_CONFIDENCE_ENGINE_H_

#include "MechanismConstants.h"
#include "EngineConfig.h"
#include "Predictor.h"
#include "Scorer.h"
#include "CoordinatePolicies.h"
#include <chrono>
#include <string>

/**
 * @brief How a motion vector was handled by the pipeline.
 */
enum class VectorDisposition {
    FirstSighting,  // No prior context; tracking anchored, confidence seeded to 1.0.
    Duplicate,      // Below the minimum transmission interval; ignored.
    Stale,          // Above the maximum interval; context purged, no verdict raised.
    Evaluated       // Both checks ran and the posterior was updated.
};

/**
 * @brief Outcome of a single onMotionVector() call.
 */
struct ConfidenceResult {
    VectorDisposition disposition = VectorDisposition::Evaluated;
    double confidence       = 1.0;   // Posterior confidence S in [0,1] for this sender.
    double distError        = 0.0;   // Prediction error epsilon [m].
    double dynamicMaxPosDev = 0.0;   // Context-aware threshold d_max [m].
    double dt               = 0.0;   // Inter-frame interval used by d_max [s].
    double safeSpeed        = 0.0;   // Sanitized speed v_safe used by d_max.
    double yawRateHat       = 0.0;   // Regressed yaw rate omega_hat used by d_max.
    bool consistencyPassed  = true;  // Physical invariance verdict.
    bool predictivePassed   = true;  // Trajectory prediction verdict.

    bool evaluated() const { return disposition == VectorDisposition::Evaluated; }
};

/**
 * @brief Per-stage latency, filled only when a timing sink is supplied.
 */
struct StageTiming {
    double predictor_us = 0.0;  // Consistency + dynamic threshold + predictive check.
    double scorer_us    = 0.0;  // Evidence dispatch and posterior update.
};

/**
 * @brief Local confidence attribution for V2X kinematic beacons.
 *
 * Composes the Predictor (Sec. IV-A) and the Scorer (Sec. IV-B) into the
 * two-stage pipeline of Fig. 1. The engine is container-agnostic: it consumes a
 * motion vector (phi, lambda, theta, v, a, omega) plus an opaque sender id, and
 * knows nothing about the beacon format that carried it.
 */
template <typename CoordPolicy = CartesianCoordinatePolicy>
class BeaconConfidenceEngine {
public:
    BeaconConfidenceEngine(const std::string& motionModelName,
                           const std::string& evaluatorName,
                           const EngineConfig& config)
        : m_cfg(config)
        , m_predictor(motionModelName, config, CoordPolicy::name())
        , m_scorer(evaluatorName, config)
    {
    }

    /**
     * @brief Process one received motion vector and return its confidence.
     *
     * @param senderId  Identifier used to route the vector to its tracking history.
     * @param mv        The received motion vector.
     * @param timing    Optional per-stage latency sink for benchmarking.
     */
    ConfidenceResult onMotionVector(long senderId,
                                    const KinematicState& mv,
                                    StageTiming* timing = nullptr)
    {
        ConfidenceResult result;

        // No prior context: anchor tracking, nothing to verify against yet.
        if (!m_predictor.isTracked(senderId)) {
            m_predictor.cacheRegister(senderId, mv);
            m_scorer.setConfidence(senderId, 1.0);
            result.disposition = VectorDisposition::FirstSighting;
            result.confidence  = 1.0;
            return result;
        }

        const auto& history = m_predictor.history(senderId);
        const KinematicState pastState = history.back();
        double dt = mv.time - pastState.time;

        result.confidence = m_scorer.confidenceOf(senderId);

        if (dt < ConstantsPhysics::MIN_INTER_FRAME_TIME) {
            result.disposition = VectorDisposition::Duplicate;
            return result;
        }
        if (dt > ConstantsPhysics::MAX_INTER_FRAME_TIME) {
            m_predictor.purge(senderId, mv);
            result.disposition = VectorDisposition::Stale;
            return result;
        }

        // --- Stage 1: Predictor + hard checks ---------------------------------
        const auto t0 = std::chrono::high_resolution_clock::now();

        PhysicalInvarianceResult physics = consistencyCheck<CoordPolicy>(
            pastState, mv, m_cfg.speedToleranceMps, m_cfg.enableConsistency);

        result.predictivePassed = predictiveCheck(
            m_predictor.model(), history, pastState, mv, dt, m_cfg, physics,
            result.distError, result.dynamicMaxPosDev,
            m_cfg.enableDynThresh, m_cfg.enablePrediction, &result.yawRateHat);

        const auto t1 = std::chrono::high_resolution_clock::now();

        result.consistencyPassed = !physics.failed;
        result.dt        = physics.dt;
        result.safeSpeed = physics.safeSpeed;

        // --- Stage 2: Scorer --------------------------------------------------
        ConfidenceContext context {
            history,
            pastState,
            mv,
            m_scorer.confidenceOf(senderId),
            result.distError,
            physics,
            result.dynamicMaxPosDev,
            senderId
        };

        const auto t2 = std::chrono::high_resolution_clock::now();
        result.confidence = m_scorer.evaluate(context);
        const auto t3 = std::chrono::high_resolution_clock::now();

        if (timing) {
            timing->predictor_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            timing->scorer_us    = std::chrono::duration<double, std::micro>(t3 - t2).count();
        }

        m_predictor.append(senderId, mv);

        result.disposition = VectorDisposition::Evaluated;
        return result;
    }

    double confidenceOf(long senderId) const { return m_scorer.confidenceOf(senderId); }
    const std::map<long, double>& confidences() const { return m_scorer.confidences(); }

    Predictor& predictor() { return m_predictor; }
    Scorer& scorer() { return m_scorer; }
    const EngineConfig& config() const { return m_cfg; }

    void reset()
    {
        m_predictor.reset();
        m_scorer.reset();
    }

private:
    EngineConfig m_cfg;
    Predictor m_predictor;
    Scorer m_scorer;
};

#endif // BEACON_CONFIDENCE_ENGINE_H_
