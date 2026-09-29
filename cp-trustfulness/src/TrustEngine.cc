#include "../include/TrustEngine.h"
#include "../include/AssociationEngines.h"
#include "../include/CheckerPipeline.h"
#include "../include/CoordinatePolicies.h"
#include "../include/PhysicalInvariance.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery {

namespace {

GeoPoint positionOf(const KinematicState& s)
{
    return GeoPoint{s.lat, s.lon};
}

KinematicState stateOf(const ObjectClaim& o)
{
    KinematicState s;
    s.lat = o.position.lat;
    s.lon = o.position.lon;
    s.speed = o.speed;
    s.heading = o.heading;
    s.time = o.time;
    return s;
}

/// A report that cannot be physical: Paper 1's single-frame penalty ceiling (instant revocation).
PhysicalInvarianceResult impossible()
{
    PhysicalInvarianceResult r;
    r.failed = true;
    r.excessRatio = std::numeric_limits<double>::infinity();
    return r;
}

}  // namespace

TrustEngine::TrustEngine(const TrustEngineParams& params, std::unique_ptr<IDeadReckoningModel> drModel,
                         std::unique_ptr<IAssociationEngine> associationEngine) :
    m_params(params),
    m_drModel(std::move(drModel)),
    m_associationEngine(std::move(associationEngine)),
    m_ldm(params.ldmRetention)
{
}

double TrustEngine::gate(double gnssSources, double sensorVariance) const
{
    const double sigma = m_params.gnss.positionSigma();
    return m_params.association.kappaGate * std::sqrt(gnssSources * sigma * sigma + sensorVariance);
}

// ----------------------------------------------------------------------------
// Plausibility: Paper 1, exactly as in the VeReMi runner (veremi-extension/veremi_runner.cc::processMessage).
// ----------------------------------------------------------------------------
void TrustEngine::plausibilityStep(std::deque<KinematicState>& history, Scores& scores, const KinematicState& state,
                                   const PhysicalInvarianceResult* violation, bool keep, double noiseVariance,
                                   Reason violationReason)
{
    scores.penaltyNow = Reason::None;
    if (violation) {
        // Evidence that does not need the track's history: dispatched as a Phase 0 failure.
        const KinematicState& past = history.empty() ? state : history.back();
        TrustEvaluationContext ctx{history, past, state, scores.plausibility, 0.0, *violation, 0.0, 0};
        scores.plausibility = m_paper1.evaluateTrust(ctx, m_params.kinematics);
        scores.penaltyNow = scores.lastPenalty = violationReason;
        if (keep) {
            history.push_back(state);
            while (history.size() > static_cast<size_t>(std::max(1, m_params.historySize))) {
                history.pop_front();
            }
        }
        return;
    }
    if (history.empty()) {
        history.push_back(state);
        return;
    }
    const KinematicState past = history.back();
    const double dt = state.time - past.time;
    if (dt < 0.05) {
        return;  // duplicate or same-instant report
    }
    if (dt > ConstantsPhysics::MAX_INTER_FRAME_TIME) {
        history.clear();  // stale context: anchor a fresh trajectory, keep the score
        history.push_back(state);
        return;
    }
    // Paper 1's tolerance absorbs GNSS velocity jitter between two CAMs (VeReMi: at most 0.15 m of position jitter).
    // Two radar measurements of a perceived object carry independent noise, which adds up to
    // kappa * sqrt(variance of their difference) / dt of apparent speed; the tolerance widens by exactly that.
    const double tolerance = m_params.kinematics.speedToleranceMps +
                             m_params.association.kappaGate * std::sqrt(std::max(0.0, noiseVariance)) / dt;
    const PhysicalInvarianceResult physics = phase0_PhysicalInvariance<Wgs84CoordinatePolicy>(past, state, tolerance);
    double error = 0.0;
    double threshold = 0.0;
    phase1_KinematicConsistency(m_drModel.get(), history, past, state, dt, m_params.kinematics, physics, error,
                                threshold);
    TrustEvaluationContext ctx{history, past, state, scores.plausibility, error, physics, threshold, 0};
    scores.plausibility = m_paper1.evaluateTrust(ctx, m_params.kinematics);
    if (physics.failed || error > threshold) {
        scores.penaltyNow = scores.lastPenalty = physics.failed ? Reason::Physics : Reason::Kinematics;
    }
    history.push_back(state);
    while (history.size() > static_cast<size_t>(std::max(1, m_params.historySize))) {
        history.pop_front();
    }
}

// ----------------------------------------------------------------------------
// Anchoring: the same counterless filter. The radar decides where it can; elsewhere peers can only confirm.
// ----------------------------------------------------------------------------
void TrustEngine::anchoringStep(Scores& scores, Verdict ego, int peers)
{
    const double w = m_params.kinematics.priorWeight > 0.0 ? m_params.kinematics.priorWeight
                                                           : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;
    double support = 0.0;
    double contradiction = 0.0;
    if (ego == Verdict::Supported) {
        support = m_params.anchorSupport;
    } else if (ego == Verdict::Contradicted) {
        contradiction = m_params.anchorPenalty;
        scores.lastContradiction = Reason::Radar;
    }
    scores.anchoringEgo = counterlessUpdate(scores.anchoringEgo, support, contradiction, w);
    if (ego == Verdict::Unverifiable && m_params.enablePeers && peers > 0) {
        support = std::min(m_params.anchorSupport, m_params.peerSupport * peers);
    }
    scores.anchoring = counterlessUpdate(scores.anchoring, support, contradiction, w);
}

ClaimState TrustEngine::classify(double plausibility, double anchoring) const
{
    if (plausibility < m_params.implausibleBelow) {
        return ClaimState::Implausible;
    }
    if (anchoring < m_params.contradictedBelow) {
        return ClaimState::Contradicted;
    }
    return anchoring >= m_params.anchoredFrom ? ClaimState::Anchored : ClaimState::Unanchored;
}

Reason TrustEngine::reasonFor(ClaimState state, const Scores& s) const
{
    if (state == ClaimState::Implausible) {
        return s.lastPenalty;
    }
    return state == ClaimState::Contradicted ? s.lastContradiction : Reason::None;
}

void TrustEngine::report(ClaimKind kind, StationId author, uint16_t objectId, uint64_t track, double now,
                         const Scores& s, Verdict ego, int peers)
{
    if (!m_sink) {
        return;
    }
    ClaimResult r;
    r.author = author;
    r.kind = kind;
    r.objectId = objectId;
    r.ldmTrack = track;
    r.time = now;
    r.plausibility = s.plausibility;
    r.anchoringEgo = s.anchoringEgo;
    r.anchoring = s.anchoring;
    r.egoVerdict = ego;
    r.peers = peers;
    r.state = classify(s.plausibility, s.anchoring);
    r.stateEgo = classify(s.plausibility, s.anchoringEgo);
    r.reason = reasonFor(r.state, s);
    r.reasonEgo = reasonFor(r.stateEgo, s);
    r.penalty = s.penaltyNow;
    m_sink(r);
}

// ----------------------------------------------------------------------------
// The receiver's radar: a detection within the gate supports the claim; no detection contradicts it only where the
// claim's whole uncertainty disc (the gate) lies inside a cone and the line of sight to it is free.
// ----------------------------------------------------------------------------
Verdict TrustEngine::observe(const GeoPoint& p, double gate, const EgoView& ego, const IOcclusionModel& statics) const
{
    double nearest = std::numeric_limits<double>::infinity();
    for (const EgoDetection& d : ego.detections) {
        nearest = std::min(nearest, Geodesy::distance(d.position, p));
    }
    if (!m_params.enableRadar) {
        return Verdict::Unverifiable;
    }
    if (nearest <= gate) {
        return Verdict::Supported;
    }
    // The claimed object may truly be anywhere within the gate: only if the radar would have seen every such
    // position does its absence contradict the claim.
    const bool certain = m_params.coverage.coversDisc(ego.centre, ego.heading, p, gate);
    if (!certain || lineBlocked(ego.centre, p, ego, statics)) {
        return Verdict::Unverifiable;
    }
    return Verdict::Contradicted;
}

/// Buildings, and anything the receiver's own radar detects between it and the point. A vehicle hiding the point
/// is nearer in the same direction, so the radar sees it; claims of other stations never occlude (a liar could
/// otherwise hide its own lies).
bool TrustEngine::lineBlocked(const GeoPoint& from, const GeoPoint& to, const EgoView& ego,
                              const IOcclusionModel& statics) const
{
    if (statics.blocked(from, to)) {
        return true;
    }
    auto toPlanar = [&](const GeoPoint& p) {
        const LocalPoint d = Geodesy::toLocal(from, p);
        return Planar::Vec2{d.east, d.north};
    };
    const Planar::Vec2 a{0.0, 0.0};
    const Planar::Vec2 b = toPlanar(to);
    for (const EgoDetection& d : ego.detections) {
        if (Planar::segmentHitsDisc(a, b, toPlanar(d.position), m_params.occluderRadius)) {
            return true;
        }
    }
    return false;
}

double TrustEngine::vehicleLength(StationId id) const
{
    auto it = m_lengths.find(id);
    return it != m_lengths.end() ? it->second : m_params.defaultVehicleLength;
}

GeoPoint TrustEngine::centreFromReference(const GeoPoint& reference, double heading, StationId id) const
{
    return Geodesy::offset(reference, heading, -0.5 * vehicleLength(id));
}

// ----------------------------------------------------------------------------
// CAM: the sender's own trajectory
// ----------------------------------------------------------------------------
void TrustEngine::onCam(const CamClaim& claim, const EgoView& ego, const IOcclusionModel& statics)
{
    const double now = ego.time;
    housekeeping(now);
    if (claim.author == ego.station) {
        return;
    }
    addEgoToMap(ego);
    if (claim.length > 0.0) {
        m_lengths[claim.author] = claim.length;
    }
    Scores& scores = m_camScores[claim.author];
    scores.lastSeen = now;
    plausibilityStep(m_camHistory[claim.author], scores, claim.state, nullptr);
    const uint64_t body = m_ldm.addCam(claim.author, claim.state);

    // sender GNSS (+ receiver GNSS) and the receiver's radar at the claimed range
    const KinematicState& c = claim.state;
    const GeoPoint claimedNow = Geodesy::project(positionOf(c), c.speed, c.heading, now - c.time);
    const double sensorVariance = m_params.radar.positionVariance(Geodesy::distance(ego.centre, claimedNow));
    const Verdict seen = observe(claimedNow, gate(1.0 + egoGnssSources(), sensorVariance), ego, statics);
    const int peers = seen == Verdict::Unverifiable
                          ? m_ldm.independentSources(body, {claim.author, ego.station}, now - m_params.inclusionPeriod,
                                                     now + 0.05)
                          : 0;
    anchoringStep(scores, seen, peers);
    report(ClaimKind::Cam, claim.author, 0, body, now, scores, seen, peers);
}

// ----------------------------------------------------------------------------
// CPM: reference pose + perceived objects
// ----------------------------------------------------------------------------

/// The CPM reference pose and the author's CAM track carry the same GNSS fix: only motion, the heading error in
/// the bumper-to-centre shift and the GNSS jitter between the two instants separate them. A larger gap is a
/// Phase 0 violation of every object the CPM places from that pose.
const PhysicalInvarianceResult* TrustEngine::poseCheck(const CpmSnapshot& cpm, const GeoPoint& authorCentre,
                                                       PhysicalInvarianceResult& out) const
{
    auto track = m_camHistory.find(cpm.author);
    if (track == m_camHistory.end() || track->second.empty()) {
        return nullptr;
    }
    // A reference already judged implausible is not used to judge other claims: a CAM trajectory with a falsified
    // speed is caught on its own track, and projecting it would copy its lie onto the CPM's real objects.
    auto scores = m_camScores.find(cpm.author);
    if (scores == m_camScores.end() || scores->second.plausibility < m_params.implausibleBelow) {
        return nullptr;
    }
    const double dt = cpm.time - track->second.back().time;
    if (dt < -0.05 || dt > m_params.maxSyncDt) {
        return nullptr;
    }
    const KinematicState projected =
        TemporalTrackSynchronizer::project(track->second, std::max(0.0, dt), m_params.syncMode, m_drModel.get());
    const double jitter = m_params.gnss.jitterFraction * m_params.gnss.positionBound;
    const double shift = 0.5 * vehicleLength(cpm.author) * m_params.gnss.headingSigma(projected.speed);
    const double variance = 2.0 * jitter * jitter + shift * shift;
    const double poseGate = SpatialAssociationUtils::computeDynamicGate(projected.speed, std::abs(dt),
                                                                        m_params.association, 0.0, variance);
    const double distance = Geodesy::distance(positionOf(projected), authorCentre);
    if (distance <= poseGate) {
        return nullptr;
    }
    out.failed = true;
    out.excessRatio = (distance - poseGate) / std::max(poseGate, 1e-3);
    return &out;
}

CpmOutcome TrustEngine::onCpm(const CpmSnapshot& cpm, const EgoView& ego, const IOcclusionModel& statics)
{
    const double now = ego.time;
    housekeeping(now);
    CpmOutcome outcome;
    if (cpm.author == ego.station) {
        return outcome;
    }
    addEgoToMap(ego);
    const StationId author = cpm.author;
    const GeoPoint authorCentre = centreFromReference(cpm.referencePosition, cpm.heading, author);
    PhysicalInvarianceResult poseResult;
    const PhysicalInvarianceResult* poseViolation =
        m_params.enablePoseCheck ? poseCheck(cpm, authorCentre, poseResult) : nullptr;

    const size_t n = cpm.objects.size();
    outcome.resolvedTo.assign(n, -1);
    outcome.admissible.assign(n, true);
    outcome.aligned.resize(n);

    // Admissibility of each report on its own, and alignment to the CPM time
    const double maxRange = m_params.coverage.maxRange() * m_params.rangeMargin;
    for (size_t j = 0; j < n; ++j) {
        const ObjectClaim& o = cpm.objects[j];
        const bool finite = std::isfinite(o.position.lat) && std::isfinite(o.position.lon) &&
                            std::isfinite(o.speed) && std::isfinite(o.heading);
        const double range = finite ? Geodesy::distance(authorCentre, o.position) : 0.0;
        outcome.admissible[j] = finite && (!m_params.enableAdmissibility ||
                                           (o.speed >= 0.0 && o.speed <= m_params.maxObjectSpeed &&
                                            (maxRange <= 0.0 || range <= maxRange) && range >= m_params.minSeparation));
        ObjectClaim aligned = o;
        if (o.hasVelocity && finite) {
            aligned.position = Geodesy::project(o.position, o.speed, o.heading, cpm.time - o.time);
        }
        aligned.time = cpm.time;
        aligned.sensorVariance = m_params.radar.positionVariance(range);
        outcome.aligned[j] = aligned;
    }

    // Resolution: which admissible objects are CAM senders (or the receiver itself)?
    std::vector<ObjectClaim> candidates;
    std::vector<size_t> candidateIndex;
    for (size_t j = 0; j < n; ++j) {
        if (outcome.admissible[j]) {
            candidates.push_back(outcome.aligned[j]);
            candidateIndex.push_back(j);
        }
    }
    std::vector<ProjectedTrack> tracks = TemporalTrackSynchronizer::synchronize(
        m_camHistory, cpm.time, m_params.syncMode, m_params.maxSyncDt, m_drModel.get(), {author, ego.station});
    // The receiver appears in other authors' CPMs but never receives its own CAM.
    ProjectedTrack self;
    self.station = ego.station;
    const GeoPoint selfAtCpm = Geodesy::project(ego.centre, ego.speed, ego.heading, cpm.time - ego.time);
    self.state.lat = selfAtCpm.lat;
    self.state.lon = selfAtCpm.lon;
    self.state.speed = ego.speed;
    self.state.heading = ego.heading;
    self.state.time = cpm.time;
    self.syncDt = std::abs(ego.time - cpm.time);
    tracks.push_back(self);
    if (m_associationEngine && !candidates.empty()) {
        AssociationContext ctx{tracks, candidates, m_params.association};
        for (const AssociationMatch& match : m_associationEngine->associate(ctx)) {
            outcome.resolvedTo[candidateIndex[match.objectIndex]] = match.station;
        }
    }

    // Each perceived object: plausibility of its track, then anchoring
    for (size_t j = 0; j < n; ++j) {
        const ObjectClaim& o = cpm.objects[j];
        ObjectTrack& t = m_objects[{author, o.objectId}];
        t.scores.lastSeen = now;
        if (!outcome.admissible[j]) {
            const PhysicalInvarianceResult violation = impossible();
            plausibilityStep(t.history, t.scores, stateOf(o), &violation, false);
            anchoringStep(t.scores, Verdict::Unverifiable, 0);
            report(ClaimKind::Object, author, o.objectId, 0, now, t.scores, Verdict::Unverifiable, 0);
            continue;
        }
        if (m_params.enableObjectKinematics) {
            plausibilityStep(t.history, t.scores, stateOf(o), poseViolation, true,
                             t.lastVariance + outcome.aligned[j].sensorVariance, Reason::Pose);
        } else if (poseViolation) {
            plausibilityStep(t.history, t.scores, stateOf(o), poseViolation, false, 0.0, Reason::Pose);
        } else {
            t.scores.penaltyNow = Reason::None;
        }
        t.lastVariance = outcome.aligned[j].sensorVariance;

        const ObjectClaim& aligned = outcome.aligned[j];
        // two GNSS fixes and two radar measurements (the author's and the one it is compared with)
        const uint64_t track =
            m_ldm.addObject(aligned, outcome.resolvedTo[j], ego.station, gate(2.0, 2.0 * aligned.sensorVariance));
        Verdict seen = Verdict::Supported;  // an object resolved to the receiver: the receiver knows where it is
        if (outcome.resolvedTo[j] != ego.station) {
            const GeoPoint atNow = aligned.hasVelocity
                                       ? Geodesy::project(aligned.position, aligned.speed, aligned.heading, now - aligned.time)
                                       : aligned.position;
            const double egoVariance = m_params.radar.positionVariance(Geodesy::distance(ego.centre, atNow));
            seen = observe(atNow, gate(1.0 + egoGnssSources(), aligned.sensorVariance + egoVariance), ego, statics);
        }
        const int peers = (seen == Verdict::Unverifiable && track)
                              ? m_ldm.independentSources(track, {author, ego.station}, now - m_params.inclusionPeriod,
                                                         now + 0.05)
                              : 0;
        anchoringStep(t.scores, seen, peers);
        report(ClaimKind::Object, author, o.objectId, track, now, t.scores, seen, peers);
    }
    return outcome;
}

void TrustEngine::tick(const EgoView& ego)
{
    housekeeping(ego.time);
    addEgoToMap(ego);
}

void TrustEngine::addEgoToMap(const EgoView& ego)
{
    if (ego.time == m_egoAddedAt) {
        return;
    }
    m_egoAddedAt = ego.time;
    for (const EgoDetection& d : ego.detections) {
        const double variance = m_params.radar.positionVariance(Geodesy::distance(ego.centre, d.position));
        m_ldm.addEgo(ego.station, d, ego.time, gate(1.0 + egoGnssSources(), 2.0 * variance));
    }
}

void TrustEngine::housekeeping(double now)
{
    m_ldm.evict(now);
    for (auto it = m_camScores.begin(); it != m_camScores.end();) {
        if (now - it->second.lastSeen > m_params.trackTimeout) {
            m_camHistory.erase(it->first);
            m_lengths.erase(it->first);
            it = m_camScores.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = m_objects.begin(); it != m_objects.end();) {
        it = (now - it->second.scores.lastSeen > m_params.trackTimeout) ? m_objects.erase(it) : std::next(it);
    }
}

}  // namespace artery
