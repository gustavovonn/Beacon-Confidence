#ifndef TRUST_ENGINE_H_
#define TRUST_ENGINE_H_

#include "Confidence.h"
#include "CpmTypes.h"
#include "ErrorModels.h"
#include "IAssociationEngine.h"
#include "IDeadReckoningModel.h"
#include "ITrustEvaluator.h"  // TrustEvaluationParams (Paper 1)
#include "Occlusion.h"
#include "SensorCoverage.h"
#include "TemporalTrackSynchronizer.h"
#include "TrustEvaluators.h"  // BetaReputationTrustEvaluator (Paper 1)
#include "WorldMap.h"
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace artery {

struct TrustEngineParams {
    // ── Components (ablation) ──
    bool enableRadar = true;             ///< the receiver's radar anchors claims (Observation)
    bool enablePeers = true;             ///< independent authors confirm (off: anchoring by the radar only)
    bool enablePoseCheck = true;         ///< CPM reference pose vs the author's CAM track
    bool enableAdmissibility = true;     ///< object reports that cannot be physical
    bool enableObjectKinematics = true;  ///< Paper 1 on perceived-object tracks
    bool enableSilence = true;           ///< missing confirmation of an unanchored CAM claim is evidence against it

    // ── Plausibility: Paper 1, unchanged (thresholds, penalty policy, prior weight) ──
    TrustEvaluationParams kinematics;
    int historySize = 4;
    double trackTimeout = 5.0;          ///< [s] forget a claim track after this silence
    double defaultVehicleLength = 5.0;  ///< [m] when the CAM carries no length

    // ── Anchoring: the same counterless filter, fed by sources the author does not control ──
    double anchorSupport = 1.0;    ///< the receiver's radar confirms (Paper 1's reward)
    double peerSupport = 0.5;      ///< per independent author confirming, at most anchorSupport in total
    /// the receiver's radar contradicts: 4 is the smallest whole penalty for which two consecutive contradictions
    /// flag a steadily anchored claim (0.667 -> 0.479 -> 0.362) and one does not (Paper 1's K = 2 policy)
    double anchorPenalty = 4.0;
    double inclusionPeriod = 1.0;  ///< [s] an honest author re-reports a tracked object at least this often
    /// Silent authors needed to contradict an unanchored CAM claim: more than one, so that one faulty author (whose
    /// radars or reported pose are wrong) cannot flag anyone; faults are independent (VeReMi / F2MD threat model).
    int minSilentAuthors = 2;

    // ── States ──
    double implausibleBelow = 0.5;   ///< Paper 1's eviction threshold on P
    double contradictedBelow = 0.4;  ///< on A: one contradiction of a fresh claim (0.5 -> 0.375), two of an anchored one
    double anchoredFrom = 0.58;      ///< on A: three consecutive confirmations from 0.5 (0.538, 0.568, 0.591)

    // ── Object admissibility (a report that cannot be physical) ──
    double maxObjectSpeed = 70.0;  ///< [m/s]
    double rangeMargin = 1.2;      ///< objects beyond coverage.maxRange() * margin are inadmissible
    double minSeparation = 1.0;    ///< [m] object centre inside the author's own body

    // ── Geometry and error models ──
    AssociationParams association;
    SyncMode syncMode = SyncMode::Ctra;
    double maxSyncDt = 1.1;        ///< [s] CAMs may be 1 s apart (EN 302 637-2)
    SensorCoverage coverage;       ///< fixed field of view, identical for every station
    GnssErrorModel gnss;
    RadarErrorModel radar;
    bool egoGnssError = true;      ///< the receiver places its own detections with its own GNSS fix
    double occluderRadius = 1.2;   ///< [m] vehicles approximated as discs
    double ldmRetention = 4.0;     ///< [s]
};

/// The confidence attributed to one message (a CAM) or one perceived object (of a CPM).
struct ClaimResult {
    StationId author = 0;
    ClaimKind kind = ClaimKind::Cam;
    uint16_t objectId = 0;         ///< CPM object id (0 for CAMs)
    uint64_t ldmTrack = 0;         ///< the physical object it was attached to (0: the receiver itself)
    double time = 0.0;             ///< receiver time [s]
    double plausibility = 1.0;     ///< P (Paper 1)
    double anchoringEgo = 0.5;     ///< A from the receiver's radar only
    double anchoring = 0.5;        ///< A from the receiver's radar and independent authors
    Verdict egoVerdict = Verdict::Unverifiable;
    int peers = 0;                 ///< independent authors confirming (counted only when the radar could not judge)
    ClaimState state = ClaimState::Unanchored;     ///< from P and A
    ClaimState stateEgo = ClaimState::Unanchored;  ///< from P and the radar-only A (ablation: no peers)
    Reason reason = Reason::None;                  ///< component behind `state` when flagged
    Reason reasonEgo = Reason::None;               ///< component behind `stateEgo` when flagged
    Reason penalty = Reason::None;                 ///< penalty applied to P by this message, if any
};

struct CpmOutcome {
    std::vector<StationId> resolvedTo;  ///< per object: station id, the receiver's id for itself, -1 unresolved
    std::vector<bool> admissible;
    std::vector<ObjectClaim> aligned;   ///< objects projected to the CPM generation time
};

/**
 * @brief Per-receiver confidence engine: attributes a confidence to every received CAM and CPM object.
 *
 *   Plausibility (P)  Paper 1 on the claim track: phase 0 (physical invariance), phase 1 (OLS-CTRA prediction
 *                     against the context-aware threshold), counterless Beta update. A CPM object that cannot be
 *                     physical, or a CPM whose reference pose contradicts its author's CAM track, is a phase 0
 *                     violation of the objects it places.
 *   Anchoring (A)     the receiver's radar confirms or contradicts the claimed position (only where it would
 *                     certainly see it: the claim's whole uncertainty disc inside a cone, line of sight free); where
 *                     it cannot judge, independent authors whose statements landed on the same LDM track confirm.
 *                     Only the receiver's own radar can contradict.
 *
 * Every claim is judged at receipt; nothing is deferred and no sender reputation is kept.
 */
class TrustEngine {
public:
    using ClaimSink = std::function<void(const ClaimResult&)>;

    TrustEngine(const TrustEngineParams& params, std::unique_ptr<IDeadReckoningModel> drModel,
                std::unique_ptr<IAssociationEngine> associationEngine);

    void setClaimSink(ClaimSink sink) { m_sink = std::move(sink); }

    void onCam(const CamClaim& claim, const EgoView& ego, const IOcclusionModel& statics);
    CpmOutcome onCpm(const CpmSnapshot& cpm, const EgoView& ego, const IOcclusionModel& statics);

    /// Keeps the LDM current with the receiver's own detections. Call periodically (e.g. every 0.2 s).
    void tick(const EgoView& ego);

    bool hasTrack(StationId id) const { return m_camHistory.count(id) > 0; }
    const TrustEngineParams& params() const { return m_params; }

    /// Gate [m] for two position estimates with `gnssSources` independent GNSS fixes and radar measurements of
    /// total per-axis variance `sensorVariance`.
    double gate(double gnssSources, double sensorVariance) const;

private:
    struct Scores {
        double plausibility = 1.0;  ///< Paper 1 starts every new sender at 1.0
        double anchoring = 0.5;
        double anchoringEgo = 0.5;
        double lastSeen = -1.0;
        Reason lastPenalty = Reason::None;  ///< what last lowered P
        Reason penaltyNow = Reason::None;   ///< what lowered P in the current message
        Reason lastContradiction = Reason::None;  ///< what last lowered A (radar or silence)
        double silenceCheckedAt = -1e9;     ///< the silence test runs at most once per inclusion period
        bool silence = false;               ///< its last verdict
    };
    struct ObjectTrack {
        std::deque<KinematicState> history;
        Scores scores;
        double lastVariance = 0.0;  ///< per-axis radar variance of the last report [m^2]
    };

    /// `noiseVariance`: per-axis variance of the difference between this report and the previous one that is not
    /// GNSS jitter (0 for CAMs; the two radar measurements for a perceived object).
    void plausibilityStep(std::deque<KinematicState>& history, Scores& scores, const KinematicState& state,
                          const PhysicalInvarianceResult* violation, bool keep = true, double noiseVariance = 0.0,
                          Reason violationReason = Reason::Impossible);
    void anchoringStep(Scores& scores, Verdict ego, int peers, bool silence = false);
    int silentAuthors(const CamClaim& claim, uint64_t body, const EgoView& ego, const IOcclusionModel& statics,
                      double now) const;
    bool peerLineBlocked(const GeoPoint& from, const GeoPoint& to, double time, const IOcclusionModel& statics) const;
    ClaimState classify(double plausibility, double anchoring) const;
    Reason reasonFor(ClaimState state, const Scores& s) const;
    void report(ClaimKind kind, StationId author, uint16_t objectId, uint64_t track, double now, const Scores& s,
                Verdict ego, int peers);

    Verdict observe(const GeoPoint& p, double gate, const EgoView& ego, const IOcclusionModel& statics) const;
    bool lineBlocked(const GeoPoint& from, const GeoPoint& to, const EgoView& ego, const IOcclusionModel& statics) const;
    double egoGnssSources() const { return m_params.egoGnssError ? 1.0 : 0.0; }
    double vehicleLength(StationId id) const;
    GeoPoint centreFromReference(const GeoPoint& reference, double heading, StationId id) const;
    const PhysicalInvarianceResult* poseCheck(const CpmSnapshot& cpm, const GeoPoint& authorCentre,
                                              PhysicalInvarianceResult& out) const;
    void addEgoToMap(const EgoView& ego);
    void housekeeping(double now);

    TrustEngineParams m_params;
    std::unique_ptr<IDeadReckoningModel> m_drModel;
    std::unique_ptr<IAssociationEngine> m_associationEngine;
    BetaReputationTrustEvaluator m_paper1;
    WorldMap m_ldm;
    ClaimSink m_sink;

    std::map<StationId, std::deque<KinematicState>> m_camHistory;  ///< CAM claim tracks (vehicle centre)
    std::map<StationId, Scores> m_camScores;
    std::map<std::pair<StationId, uint16_t>, ObjectTrack> m_objects;  ///< (author, CPM object id)
    std::map<StationId, double> m_lengths;
    double m_egoAddedAt = -1.0;
};

}  // namespace artery

#endif  // TRUST_ENGINE_H_
