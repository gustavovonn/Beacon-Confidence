#ifndef OT_OBJECT_CONFIDENCE_ENGINE_H_
#define OT_OBJECT_CONFIDENCE_ENGINE_H_

/**
 * Receiver-local object-trajectory confidence.
 *
 * One entry point, onMotionVectors(), taking a WHOLE message at once. The batch matters: a CPM
 * carries many objects and the assignment is a joint decision over all of them against all tracks.
 * Feeding them one at a time silently turns global nearest-neighbour into greedy first-fit, which
 * is precisely the configuration that swaps identities when two objects cross.
 *
 * Six stages, each a named method so a timer can wrap it and the cost report is per stage:
 *   1 plausibility      one vector against an absolute bound, no history needed   O(m)
 *   2 projectKnownObjects  predict every track to this message's time, ONCE       O(N)
 *   3 associate         gate on Mahalanobis, then joint assignment                O(k^3)
 *   4 consistency + kalmanUpdate, then computeTrajectoryConfidence
 *   5 initiateTracks    whatever matched nothing
 *   6 retireStaleTracks
 */

#include "MeasurementSpace.h"
#include "MotionVector.h"
#include "ObjectTrack.h"
#include "StageTimer.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace ot {

struct EngineParams {
    /// Gate on the squared Mahalanobis distance. 9.49 is the chi-square critical value for the 4
    /// measured dimensions (px, py, v, psi) at 95 %: "accept anything inside the 95 % ellipsoid".
    double gate = 9.49;

    /// How much the filter distrusts one report. See MeasurementNoise for why it must stay > 0.
    MeasurementNoise noise{};

    double maxSpeed = 70.0;          ///< [m/s] plausibility bound
    double initialPosVar = 25.0;     ///< [m^2] doubt a brand-new track starts with
    double initialVelVar = 25.0;
    double initialHeadVar = 1.0;
    double initialRateVar = 1.0;
    double initialAccelVar = 4.0;   ///< [(m/s^2)^2] a new track knows nothing about acceleration

    double retireAfter = 2.0;        ///< [s] without a fused report
    int confirmAfter = 3;            ///< hits before a track counts as real

    /// Ablation: skip the cheap per-axis bounds and go straight to the full ellipsoidal test.
    /// The coarse gate is an optimisation and provably cannot change which pairs are gated, so
    /// turning it off must leave every result identical and only cost time. That is worth being
    /// able to demonstrate rather than assert.
    bool coarseGate = true;

    /// Classes must match to associate. Free discrimination the SmartData motion vector already
    /// carries, and it is what stops a crossing pedestrian stealing a truck's track.
    bool classMustMatch = true;
};

class ObjectConfidenceEngine {
public:
    explicit ObjectConfidenceEngine(EngineParams p = {}) : m_params(p) {}

    /// One received message, already parsed to motion vectors. Returns one record per vector.
    std::vector<VectorRecord> onMotionVectors(const std::vector<MotionVector>& vectors,
                                              uint64_t receiver, double now);

    const std::vector<ObjectTrack>& tracks() const { return m_tracks; }
    const EngineParams& params() const { return m_params; }

    /// Swap in a different confidence policy without touching the pipeline.
    using ConfidenceFn = std::function<double(ObjectTrack&, const MotionVector&,
                                              const VectorRecord&)>;
    void setConfidencePolicy(ConfidenceFn fn) { m_confidence = std::move(fn); }

    /// Attach a timer to get a per-stage breakdown of the real pipeline. Null (the default) costs
    /// one branch per stage and nothing else. The cost evaluator is the only expected caller.
    void setStageTimer(StageTimer* t) { m_timer = t; }

    // --- the six stages, public so the cost evaluator can time them individually ---
    std::vector<const MotionVector*> plausibilityChecks(const std::vector<MotionVector>& in) const;
    void projectKnownObjects(double now);

    struct Assignment {
        std::vector<std::size_t> trackForVector;  ///< Hungarian::kNone when unmatched
        std::vector<double> cost;                 ///< normalised gap of the chosen pair
        std::vector<double> runnerUp;             ///< second-best, for the contested signal
        std::vector<int> gated;                   ///< k per vector
    };

    Assignment associate(const std::vector<const MotionVector*>& v) const;
    bool kalmanUpdate(ObjectTrack& t, const MotionVector& v);
    void initiateTrack(const MotionVector& v, double now);
    void retireStaleTracks(double now);

private:
    static Measurement measurementOf(const MotionVector& v);
    double defaultConfidence(ObjectTrack& t, const MotionVector& v,
                             const VectorRecord& r) const;

    EngineParams m_params;
    std::vector<ObjectTrack> m_tracks;
    uint64_t m_nextId = 1;
    double m_lastProjection = -1.0;
    ConfidenceFn m_confidence;
    StageTimer* m_timer = nullptr;
};

}  // namespace ot

#endif  // OT_OBJECT_CONFIDENCE_ENGINE_H_
