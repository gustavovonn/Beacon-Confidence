#ifndef CP_TRUST_ENGINE_H_
#define CP_TRUST_ENGINE_H_

#include "Confidence.h"
#include "CpmTypes.h"
#include "ErrorModels.h"
#include "IAssociationEngine.h"
#include "IDeadReckoningModel.h"
#include "ITrustEvaluator.h"  // TrustEvaluationParams (Paper 1)
#include "TemporalTrackSynchronizer.h"

#include "WorldMap.h"
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace artery {

struct CpTrustEngineParams {
    // ── Components (ablation) ──
    // Ablation: the receiver ignores CPMs entirely — no object streams, no pose check, and so no
    // corroboration for CAMs either, since a CAM is confirmed by objects that resolve to its sender.
    // CPMs are still transmitted, so channel load and the reception pattern are unchanged: this isolates
    // what the mechanism does with CPM content from what CPM traffic does to the radio.
    bool enableCpm = true;
    bool enablePeers = true;             // corroboration by other stations on the same LDM track
    bool enablePoseCheck = true;         // CPM reference pose vs the author's CAM stream
    bool enableAdmissibility = true;     // impossible vectors (non-finite, too fast, out of range, inside author)
    bool enableObjectKinematics = true;  // Paper 1 on perceived-object streams

    // ── Plausibility checks: Paper 1, unchanged (thresholds, evidence policy, prior weight) ──
    // Fitted on CAM streams (scripts/context_tuning.py). The evidence policy and prior weight are shared;
    // only the d_max coefficients differ between the two kinds.
    TrustEvaluationParams kinematics;
    // Fitted on perceived-object streams, and the only fit we adopt: `kinematics` above keeps Paper 1's
    // published CAM values because refitting them cost honest FPR for no F1 (see the NED for the numbers).
    // The object intercept comes out at 15.38 m against Paper 1's 30.85 m -- TIGHTER, not looser, because
    // Paper 1's CAM fit was dominated by motorway manoeuvres over a 45 s window on the full LuST map, while
    // perceived objects are reported inside a 1.61 km^2 urban square. `PerceivedObject` carries no yaw rate,
    // so the yaw term is unfittable here and is held at 0; that is also why a POOLED fit is not merely worse
    // but invalid -- with yaw identically 0 on objects, the yaw regressor degenerates into a CAM/object
    // indicator and its coefficient turns negative.
    TrustEvaluationParams objectKinematics;
    int historySize = 4;                // W: last vectors kept per claim stream
    // Below this gap the vector is not a NEW report (same instant, duplicate, out of order): dropped, no
    // evidence, not appended. A sameness test, not a rate bound; must stay > 0 because the CTRA predictor
    // returns early on dt <= 0 and leaves a default state thousands of km away.
    double minReportInterval = 0.001;
    // Below this gap phase 0 is skipped, but the vector is still real: phase 1 runs and it is appended.
    // Phase 0 infers speed from displacement over dt and needs a usable baseline; phase 1 does not.
    // T_GenCamMin / T_GenCpmMin = 100 ms grounds it for a SINGLE-SOURCE stream only: a CPM at t = 0.20 s
    // and a CAM at t = 0.23 s are both compliant and 30 ms apart.
    double minSpeedCheckInterval = 0.1;
    double trackTimeout = 5.0;          // [s] forget a claim stream after this silence
    double defaultVehicleLength = 5.0;  // [m] when the CAM carries no length

    // ── Corroboration: evidence in favour only, from other stations on the same LDM track ──
    double maxSupport = 1.0;       // Paper 1's reward: the most corroboration one vector can gain
    double peerSupport = 0.5;      // per confirming station, at most maxSupport in total
    double inclusionPeriod = 1.0;  // [s] window in which other stations' admitted vectors confirm

    double implausibleBelow = 0.5;   // Paper 1's threshold on the confidence

    // ── Impossible vectors ──
    double maxSpeed = 70.0;        // [m/s] CAM and object vectors
    /// [m] objects farther from their author are impossible: the authors' perception range (100 m, ITU-R M.2057-1
    /// Radar B, the sensor that generates the simulated CPMs) with a 20 % margin
    double maxObjectRange = 120.0;
    double minSeparation = 1.0;    // [m] object centre inside the author's own body

    // ── Geometry and error models ──
    AssociationParams association;
    SyncMode syncMode = SyncMode::Ctra;
    double maxSyncDt = 1.1;        // [s] CAMs may be 1.1 s apart (EN 302 637-2)
    GnssErrorModel gnss;
    RadarErrorModel radar;         // error of the authors' sensors, i.e. of CPM object positions
    double trackLifetime = 1.1;    // [s] an LDM object track nobody feeds is dropped (longest CAM gap)
};

/**
 * One phase-0/phase-1 observation, for OFFLINE THRESHOLD TUNING only. The confidence pipeline never reads
 * it. `d_max = maxPosDev + contextSpeedWeight*(v*dt) + contextYawWeight*(|w|*v)` was fitted on CAM streams
 * alone (Paper 1), yet `check()` applies it unchanged to perceived-object streams, which have a different
 * dt distribution, radar noise instead of GNSS, and a different history. This records the inputs and the
 * outcome per stream kind so the fit can be redone over CAM and CPM together.
 */
struct ContextSample {
    ClaimKind kind = ClaimKind::Cam;
    StationId author = 0;
    uint16_t objectId = 0;
    double time = 0.0;           // receiver time [s]
    double dt = 0.0;             // gap to the previous vector of this stream [s]
    double speed = 0.0;          // reported speed [m/s]
    double yawRate = 0.0;        // reported yaw rate [rad/s]
    double distError = 0.0;      // phase 1: OLS-CTRA prediction error [m]
    double threshold = 0.0;      // phase 1: d_max as currently parameterised [m]
    double noiseVariance = 0.0;  // per-axis measurement variance fed to phase 0 [m^2]
    bool physicsFailed = false;  // phase 0 fired
};

/// The confidence output with one motion vector (a CAM, or one perceived object of a CPM).
struct ClaimResult {
    StationId author = 0;
    ClaimKind kind = ClaimKind::Cam;
    uint16_t objectId = 0;         // CPM object id (0 for CAMs)
    uint64_t ldmTrack = 0;         // the object track it was admitted to (0: not admitted, or the receiver itself)
    double time = 0.0;             // receiver time [s]
    double confidence = 1.0;       // S of its claim stream after this vector
    double confidenceBase = 1.0;   // the same without corroboration evidence (evaluation only)
    int peers = 0;                 // n: other stations confirming it
    bool admitted = false;         // violated no check and entered the LDM
    ClaimState state = ClaimState::Plausible;      // from `confidence`
    ClaimState stateBase = ClaimState::Plausible;  // from `confidenceBase`
    Reason reason = Reason::None;                  // check behind the last penalty, when implausible
    Reason penalty = Reason::None;                 // penalty applied by this vector, if any
};

/**
 * The receiver's outcome for ONE perceived object of a CPM. One struct per object rather than a set of
 * parallel vectors: the index was the object's identity across six sequences that had to stay the same
 * length, and the service had to guard against them drifting apart — a guard that would have silently
 * processed fewer objects rather than reporting the fault. `aligned` already carries `author` and
 * `objectId`, so a consumer needs nothing but this.
 */
struct ObjectOutcome {
    ObjectClaim aligned;        // the object projected to the CPM generation time; carries author + objectId
    StationId resolvedTo = -1;  // the station it was resolved to; the receiver's own id for itself, -1 unresolved
    double residual = -1.0;     // the accepted association residual / its gate; DIAGNOSTIC ONLY, never grades
    int peers = 0;              // n: other stations confirming it at its update (evaluation)
    Reason plausibility = Reason::None;  // which single-message bound failed, None if it passed
    bool implausible() const { return plausibility != Reason::None; }
    bool admitted = false;      // violated no check: entered the hypothesis cache
};

/**
 * @brief Per-receiver confidence engine: one confidence per claim stream, output with every motion vector.
 *
 * Per vector: the checks (impossible vector, CPM reference pose, Paper 1's phases 0 and 1 on the stream's history)
 * give evidence for or against it. A vector that violates no check is admitted to the LDM, where the other stations
 * that report the same object corroborate it (evidence in favour only). One counterless Beta update of the stream's
 * confidence takes both. Nothing is deferred, and no score is kept per station or per physical object.
 */
class CpTrustEngine {
public:
    using ClaimSink = std::function<void(const ClaimResult&)>;
    using ContextSink = std::function<void(const ContextSample&)>;

    CpTrustEngine(const CpTrustEngineParams& params, std::unique_ptr<IDeadReckoningModel> drModel,
                std::unique_ptr<IAssociationEngine> associationEngine);

    void setClaimSink(ClaimSink sink) { m_sink = std::move(sink); }
    /// Optional, evaluation only: every evaluated phase-0/phase-1 comparison, for threshold tuning.
    void setContextSink(ContextSink sink) { m_contextSink = std::move(sink); }

    void onCam(const CamClaim& claim, const EgoView& ego);
    std::vector<ObjectOutcome> onCpm(const CpmSnapshot& cpm, const EgoView& ego);

    bool hasTrack(StationId id) const { return m_camHistory.count(id) > 0; }
    const CpTrustEngineParams& params() const { return m_params; }

    /// Gate [m] for two position estimates with `gnssSources` independent GNSS fixes and sensor measurements of
    /// total per-axis variance `sensorVariance`.
    double gate(double gnssSources, double sensorVariance) const;

private:
    struct Score {
        double confidence = 1.0;      // S: Paper 1 starts every new stream at 1.0
        double confidenceBase = 1.0;  // S without corroboration (evaluation only)
        double lastSeen = -1.0;
        Reason lastPenalty = Reason::None;  // what last lowered S
        Reason penaltyNow = Reason::None;   // what lowered S with the current vector
    };
    /// A claim stream of perceived objects: the successive reports of ONE author about ONE of its objects.
    /// It carries its own trajectory, because that is what its plausibility is judged against: across one
    /// author's reports of one object the author's positioning bias is constant and cancels, exactly as it
    /// does for a CAM, leaving only the radar noise of the two measurements.
    struct ObjectStream {
        Score scores;
        std::deque<KinematicState> history;  // last W vectors of this (author, object) stream
        double lastVariance = 0.0;           // per-axis sensor variance of the last vector [m^2]
    };
    /// Evidence of the checks for one vector (Paper 1's dispatch).
    struct Evidence {
        bool evaluated = false;  // false: first vector of a stream, or a duplicate: no evidence, no update
        bool violation = false;  // impossible, pose mismatch, phase 0 failure or phase 1 excess beyond the soft margin
        double positive = 0.0;
        double negative = 0.0;
        Reason reason = Reason::None;
    };

    /// Checks of one vector against its stream's history (appended unless `keep` is false). `violation`: evidence
    /// that needs no history (impossible vector, pose mismatch), dispatched as a phase 0 failure. `noiseVariance`:
    /// per-axis variance of the difference between this vector and the previous one that is not GNSS jitter
    /// (0 for CAMs; the two sensor measurements for a perceived object).
    Evidence check(std::deque<KinematicState>& history,
                   const KinematicState& state,
                   const PhysicalInvarianceResult* violation,
                   Reason violationReason,
                   bool keep = true,
                   double noiseVariance = 0.0,
                   ContextSample* ctx = nullptr,
                   const TrustEvaluationParams* kin = nullptr) const;

    Evidence dispatch(const PhysicalInvarianceResult& physics, double error, double threshold) const;

    void updateScore(Score& score, const Evidence& ev, int peers);

    bool plausible(const Score& s) const { return s.confidence >= m_params.implausibleBelow; }

    void report(ClaimKind kind, StationId author, uint16_t objectId, uint64_t track, double now, const Score& s,
                int peers, bool admitted);

    double vehicleLength(StationId id) const;
    /**
     * Per-axis variance of the position difference between two consecutive self-reports of one station
     * that is NOT real motion. Derived from the cited GNSS model [Kamel et al., IEEE ICC 2020]:
     * `E_k ~ N((E_0 + E_{k-1})/2, (c*E_0)^2)` is an AR(1) in `d = E - E_0` with coefficient 1/2, so its
     * stationary variance is `Var(eps)/(1 - 1/4)`, and the difference of two stationary samples asymptotes
     * to `2/(1 - 1/4) = 8/3` times `Var(eps)`. With `Var(eps) = (c*E_0)^2` bounded over `E_0 ~ U(-b, b)`
     * by `(c*b)^2`, the worst case is `(8/3)*(c*b)^2` = 0.060 m^2 for c = 0.03 and b = 5 m.
     *
     * Phase 0 divides displacement by dt, so without this term its flat 4 m/s tolerance is only valid at
     * long dt: 0.15 m of jitter reads as 1.5 m/s at dt = 0.1 s. The term makes the tolerance scale as
     * `4.0 + kappa*sqrt(var)/dt`, which is how the perceived-object path already budgets its radar noise.
     */
    double selfReportJitterVariance() const
    {
        const double jitter = m_params.gnss.jitterFraction * m_params.gnss.positionBound;
        return (8.0 / 3.0) * jitter * jitter;
    }
    GeoPoint centreFromReference(const GeoPoint& reference, double heading, StationId id) const;

    double poseCheck(const CpmSnapshot& cpm, const GeoPoint& authorCentre) const;

    std::vector<ProjectedTrack> projectedBodies(double targetTime, const std::set<StationId>& excluded) const;

    /**
     * PLAUSIBILITY for a self-report: the single-message bounds, as one wrapper so the cost analysis has
     * one timing boundary rather than one per bound. Returns which bound failed, or `Reason::None`.
     * A CAM has no sensing geometry, so its only absolute bound is speed; a value that cannot be
     * evaluated at all fails that bound vacuously.
     */
    Reason screenCam(const CamClaim& claim) const;
    /// Fuses into the body track, counts corroboration and scores. The counterpart of `admitObjects`.
    void admitCam(const CamClaim& claim, const EgoView& ego, Score& scores, const Evidence& ev, double now);

    /// Grades the declared reference pose against the author's own CAM track and charges it to the
    /// author's stream. The returned evidence is what the objects of this CPM inherit.
    Evidence gradeReferencePose(const CpmSnapshot& cpm, const GeoPoint& authorCentre, double now);
    /// Check 0 on every object, plus alignment to the CPM time. Sizes the outcome.
    void screenObjects(const CpmSnapshot& cpm, const GeoPoint& authorCentre, std::vector<ObjectOutcome>& outcome) const;
    /// Charges and drops the vectors check 0 rejected.
    void penaliseImpossibleObjects(const CpmSnapshot& cpm, const std::vector<ObjectOutcome>& outcome, double now);
    /// Assigns the surviving objects to known bodies, filling `resolvedTo` and `residual`.
    void resolveObjects(const CpmSnapshot& cpm, const EgoView& ego, std::vector<ObjectOutcome>& outcome);
    /// Check 1 from the residual; charges whatever violates and decides admission.
    std::vector<Evidence> gradeObjects(const CpmSnapshot& cpm, const Evidence& pose, std::vector<ObjectOutcome>& outcome,
                                       double now);

    /// Fuses the admitted objects into their tracks, counts corroboration and scores them.
    void admitObjects(const CpmSnapshot& cpm, const EgoView& ego, std::vector<ObjectOutcome>& outcome,
                      const std::vector<Evidence>& evidence, double now);

    void housekeeping(double now);

    CpTrustEngineParams m_params;
    std::unique_ptr<IDeadReckoningModel> m_drModel;
    std::unique_ptr<IAssociationEngine> m_associationEngine;
    WorldMap m_ldm;
    ClaimSink m_sink;
    ContextSink m_contextSink;

    std::map<StationId, Score> m_camScores;
    std::map<StationId, std::deque<KinematicState>> m_camHistory;
    std::map<std::pair<StationId, uint16_t>, ObjectStream> m_objects;
    std::map<StationId, double> m_lengths;
};

}  // namespace artery

#endif  // CP_TRUST_ENGINE_H_
