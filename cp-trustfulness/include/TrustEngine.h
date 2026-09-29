#ifndef TRUST_ENGINE_H_
#define TRUST_ENGINE_H_

#include "Confidence.h"
#include "CpmTypes.h"
#include "ErrorModels.h"
#include "IAssociationEngine.h"
#include "IDeadReckoningModel.h"
#include "ITrustEvaluator.h"  // TrustEvaluationParams (Paper 1)
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
    bool enablePeers = true;             ///< independent authors reporting the same object confirm it
    bool enablePoseCheck = true;         ///< CPM reference pose vs the author's CAM track
    bool enableAdmissibility = true;     ///< object reports that cannot be physical
    bool enableObjectKinematics = true;  ///< Paper 1 on perceived-object tracks

    // ── Plausibility: Paper 1, unchanged (thresholds, penalty policy, prior weight) ──
    TrustEvaluationParams kinematics;
    int historySize = 4;
    double trackTimeout = 5.0;          ///< [s] forget a claim track after this silence
    double defaultVehicleLength = 5.0;  ///< [m] when the CAM carries no length

    // ── Corroboration: the same counterless filter, fed only by other authors (it can only confirm) ──
    double maxSupport = 1.0;       ///< Paper 1's reward: the most one message can gain
    double peerSupport = 0.5;      ///< per independent author confirming, at most maxSupport in total
    double inclusionPeriod = 1.0;  ///< [s] window in which other authors' reports of the same object confirm a claim

    // ── States ──
    double implausibleBelow = 0.5;   ///< Paper 1's eviction threshold on P
    double corroboratedFrom = 0.58;  ///< on C: three consecutive full confirmations from 0.5 (0.538, 0.568, 0.591)

    // ── Object admissibility (a report that cannot be physical) ──
    double maxObjectSpeed = 70.0;  ///< [m/s]
    /// [m] objects farther from their author are inadmissible: the authors' perception range (100 m, ITU-R M.2057-1
    /// Radar B, the sensor that generates the simulated CPMs) with a 20 % margin
    double maxObjectRange = 120.0;
    double minSeparation = 1.0;    ///< [m] object centre inside the author's own body

    // ── Geometry and error models ──
    AssociationParams association;
    SyncMode syncMode = SyncMode::Ctra;
    double maxSyncDt = 1.1;        ///< [s] CAMs may be 1 s apart (EN 302 637-2)
    GnssErrorModel gnss;
    RadarErrorModel radar;         ///< error of the authors' sensors, i.e. of CPM object positions
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
    double corroboration = 0.5;    ///< C (other authors)
    int peers = 0;                 ///< independent authors confirming this message
    ClaimState state = ClaimState::Uncorroborated;  ///< from P and C
    Reason reason = Reason::None;                   ///< component behind `state` when implausible
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
 *   Corroboration (C) the same filter, fed by independent authors whose statements landed on the same LDM track.
 *                     It only confirms: no vehicle can lower the confidence of another.
 *
 * Every claim is judged at receipt; nothing is deferred and no sender reputation is kept.
 */
class TrustEngine {
public:
    using ClaimSink = std::function<void(const ClaimResult&)>;

    TrustEngine(const TrustEngineParams& params, std::unique_ptr<IDeadReckoningModel> drModel,
                std::unique_ptr<IAssociationEngine> associationEngine);

    void setClaimSink(ClaimSink sink) { m_sink = std::move(sink); }

    void onCam(const CamClaim& claim, const EgoView& ego);
    CpmOutcome onCpm(const CpmSnapshot& cpm, const EgoView& ego);

    bool hasTrack(StationId id) const { return m_camHistory.count(id) > 0; }
    const TrustEngineParams& params() const { return m_params; }

    /// Gate [m] for two position estimates with `gnssSources` independent GNSS fixes and sensor measurements of
    /// total per-axis variance `sensorVariance`.
    double gate(double gnssSources, double sensorVariance) const;

private:
    struct Scores {
        double plausibility = 1.0;  ///< Paper 1 starts every new sender at 1.0
        double corroboration = 0.5;
        double lastSeen = -1.0;
        Reason lastPenalty = Reason::None;  ///< what last lowered P
        Reason penaltyNow = Reason::None;   ///< what lowered P in the current message
    };
    struct ObjectTrack {
        std::deque<KinematicState> history;
        Scores scores;
        double lastVariance = 0.0;  ///< per-axis sensor variance of the last report [m^2]
    };

    /// `noiseVariance`: per-axis variance of the difference between this report and the previous one that is not
    /// GNSS jitter (0 for CAMs; the two sensor measurements for a perceived object).
    void plausibilityStep(std::deque<KinematicState>& history, Scores& scores, const KinematicState& state,
                          const PhysicalInvarianceResult* violation, bool keep = true, double noiseVariance = 0.0,
                          Reason violationReason = Reason::Impossible);
    void corroborationStep(Scores& scores, int peers);
    ClaimState classify(double plausibility, double corroboration) const;
    void report(ClaimKind kind, StationId author, uint16_t objectId, uint64_t track, double now, const Scores& s,
                int peers);

    double vehicleLength(StationId id) const;
    GeoPoint centreFromReference(const GeoPoint& reference, double heading, StationId id) const;
    const PhysicalInvarianceResult* poseCheck(const CpmSnapshot& cpm, const GeoPoint& authorCentre,
                                              PhysicalInvarianceResult& out) const;
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
};

}  // namespace artery

#endif  // TRUST_ENGINE_H_
