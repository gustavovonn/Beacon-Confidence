#include "../include/CpTrustEngine.h"
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

CpTrustEngine::CpTrustEngine(const CpTrustEngineParams& params, std::unique_ptr<IDeadReckoningModel> drModel,
                         std::unique_ptr<IAssociationEngine> associationEngine) :
    m_params(params),
    m_drModel(std::move(drModel)),
    m_associationEngine(std::move(associationEngine)),
    m_ldm(params.trackLifetime)
{
}

double CpTrustEngine::gate(double gnssSources, double sensorVariance) const
{
    const double sigma = m_params.gnss.positionSigma();
    return m_params.association.kappaGate * std::sqrt(gnssSources * sigma * sigma + sensorVariance);
}

// ----------------------------------------------------------------------------
// Checks: Paper 1, exactly as in the VeReMi runner (veremi-extension/veremi_runner.cc::processMessage). The evidence
// is returned instead of applied, so that corroboration can join it in one update.
// ----------------------------------------------------------------------------

/// Paper 1's evidence dispatch (BetaReputationTrustEvaluator): phase 0 failure or severe phase 1 excess are
/// violations; a marginal excess (up to softMarginRatio) is a small penalty; a pass is the reward.
CpTrustEngine::Evidence CpTrustEngine::dispatch(const PhysicalInvarianceResult& physics, double error, double threshold) const
{
    const TrustEvaluationParams& k = m_params.kinematics;
    Evidence ev;
    ev.evaluated = true;
    if (physics.failed) {
        ev.violation = true;
        ev.negative = std::min(k.maxBetaPenalty, k.penaltySevereBase + k.penaltySevereScale * physics.excessRatio);
    } else if (error <= threshold) {
        ev.positive = k.alphaReward;
    } else if (error <= threshold * k.softMarginRatio) {
        ev.negative = k.penaltySoftBase + k.penaltySoftScale * (error - threshold) / threshold;
    } else {
        ev.violation = true;
        ev.negative = std::min(k.maxBetaPenalty, k.penaltySevereBase + k.penaltySevereScale * (error - threshold) / threshold);
    }
    return ev;
}

CpTrustEngine::Evidence CpTrustEngine::check(std::deque<KinematicState>& history,
                                             const KinematicState& state,
                                             const PhysicalInvarianceResult* violation,
                                             Reason violationReason,
                                             bool keep,
                                             double noiseVariance,
                                             ContextSample* ctx,
                                             const TrustEvaluationParams* kin) const
{
    const TrustEvaluationParams& k = kin ? *kin : m_params.kinematics;
    auto append = [&]() {
        history.push_back(state);
        while (history.size() > static_cast<size_t>(std::max(1, m_params.historySize))) {
            history.pop_front();
        }
    };
    if (violation) {
        // Evidence that needs no history: dispatched as a phase 0 failure.
        Evidence ev = dispatch(*violation, 0.0, 0.0);
        ev.reason = violationReason;
        if (keep) {
            append();
        }
        return ev;
    }
    if (history.empty()) {
        history.push_back(state);  // first vector of a stream: nothing to compare with
        return Evidence{};
    }
    const KinematicState past = history.back();
    const double dt = state.time - past.time;
    if (dt < m_params.minReportInterval) {
        return Evidence{};  // not a new report: same instant, duplicate, or out of order
    }
    if (dt > ConstantsPhysics::MAX_INTER_FRAME_TIME) {
        history.clear();  // stale context: anchor a fresh trajectory, keep the score
        history.push_back(state);
        return Evidence{};
    }
    // Two sensor measurements of a perceived object carry independent noise, which adds up to
    // kappa * sqrt(variance of their difference) / dt of apparent speed; the tolerance widens by exactly that.
    const double tolerance = k.speedToleranceMps +
                             m_params.association.kappaGate * std::sqrt(std::max(0.0, noiseVariance)) / dt;

    const PhysicalInvarianceResult physics =
        phase0_PhysicalInvariance<Wgs84CoordinatePolicy>(past, state, tolerance, true, m_params.minSpeedCheckInterval);

    double error = 0.0;
    double threshold = 0.0;

    phase1_KinematicConsistency(m_drModel.get(), history, past, state, dt, k, physics, error, threshold);

    Evidence ev = dispatch(physics, error, threshold);
    if (ev.negative > 0.0) {
        ev.reason = physics.failed ? Reason::PosSpeedConsistency : Reason::TrajectoryConsistency;
    }
    if (ctx) {  // tuning only; the caller fills in the identity and forwards it
        ctx->dt = dt;
        ctx->speed = state.speed;
        ctx->yawRate = state.yawRate;
        ctx->distError = error;
        ctx->threshold = threshold;
        ctx->noiseVariance = noiseVariance;
        ctx->physicsFailed = physics.failed;
    }
    append();
    return ev;
}

// ----------------------------------------------------------------------------
// Scorer: one counterless Beta update per vector with the evidence of the checks and, for an admitted vector, the
// corroboration of `peers` other stations (evidence in favour only, capped at Paper 1's reward). The same update
// without corroboration is kept for evaluation. A vector the checks could not evaluate leaves both unchanged.
// ----------------------------------------------------------------------------
void CpTrustEngine::updateScore(Score& score, const Evidence& ev, int peers)
{
    score.penaltyNow = Reason::None;
    if (!ev.evaluated) {
        return;
    }
    const double w = m_params.kinematics.priorWeight > 0.0 ? m_params.kinematics.priorWeight
                                                           : ConstantsPhysics::DEFAULT_PRIOR_WEIGHT;

    const double corroboration =
        m_params.enablePeers ? std::min(m_params.maxSupport, m_params.peerSupport * std::max(0, peers)) : 0.0;

    score.confidence = counterlessUpdate(score.confidence, ev.positive + corroboration, ev.negative, w);

    score.confidenceBase = counterlessUpdate(score.confidenceBase, ev.positive, ev.negative, w);

    if (ev.negative > 0.0) {
        score.penaltyNow = score.lastPenalty = ev.reason;
    }
}

void CpTrustEngine::report(ClaimKind kind, StationId author, uint16_t objectId, uint64_t track, double now,
                         const Score& s, int peers, bool admitted)
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
    r.confidence = s.confidence;
    r.confidenceBase = s.confidenceBase;
    r.peers = peers;
    r.admitted = admitted;
    r.state = s.confidence < m_params.implausibleBelow ? ClaimState::Implausible : ClaimState::Plausible;
    r.stateBase = s.confidenceBase < m_params.implausibleBelow ? ClaimState::Implausible : ClaimState::Plausible;
    r.reason = (r.state == ClaimState::Implausible || r.stateBase == ClaimState::Implausible) ? s.lastPenalty
                                                                                               : Reason::None;
    r.penalty = s.penaltyNow;
    m_sink(r);
}

double CpTrustEngine::vehicleLength(StationId id) const
{
    auto it = m_lengths.find(id);
    return it != m_lengths.end() ? it->second : m_params.defaultVehicleLength;
}

GeoPoint CpTrustEngine::centreFromReference(const GeoPoint& reference, double heading, StationId id) const
{
    return Geodesy::offset(reference, heading, -0.5 * vehicleLength(id));
}

Reason CpTrustEngine::screenCam(const CamClaim& claim) const
{
    const KinematicState& c = claim.state;

    const bool finite =
        std::isfinite(c.lat) && std::isfinite(c.lon) && std::isfinite(c.speed) && std::isfinite(c.heading);

    const bool tooFast = m_params.enableAdmissibility && (c.speed < 0.0 || c.speed > m_params.maxSpeed);

    return (finite && !tooFast) ? Reason::None : Reason::SpeedPlausibility;
}

/// Admission, the one stage that is genuinely identical for a self-report and for a perceived object:
/// fuse into the track, count the other stations on it, and score once with both kinds of evidence.
void CpTrustEngine::admitCam(const CamClaim& claim, const EgoView& ego, Score& scores, const Evidence& ev,
                           double now)
{
    const uint64_t body = m_ldm.addCam(claim.author, claim.state);

    // The sender is excluded from its own corroboration count, so a vector can never corroborate itself
    // however the hypothesis was updated a moment ago.
    const int peers =
        m_ldm.independentSources(body, {claim.author, ego.station}, now - m_params.inclusionPeriod, now + 0.05);

    updateScore(scores, ev, peers);
    report(ClaimKind::Cam, claim.author, 0, body, now, scores, peers, true);
}

void CpTrustEngine::onCam(const CamClaim& claim, const EgoView& ego)
{
    const double now = ego.time;
    housekeeping(now);
    if (claim.author == ego.station) {
        return;  // the receiver never judges its own message
    }
    if (claim.length > 0.0) {
        m_lengths[claim.author] = claim.length;
    }
    Score& scores = m_camScores[claim.author];
    scores.lastSeen = now;

    if (const Reason bound = screenCam(claim); bound != Reason::None) {
        // A plausibility failure takes the largest penalty and is dropped, not even kept in the history.
        const PhysicalInvarianceResult violation = impossible();
        Evidence ev = dispatch(violation, 0.0, 0.0);
        ev.reason = bound;
        // charged and stopped: no track, no corroboration
        updateScore(scores, ev, 0);
        report(ClaimKind::Cam, claim.author, 0, 0, now, scores, 0, false);
        return;
    }

    ContextSample ctx;
    const Evidence ev = check(m_camHistory[claim.author], claim.state, nullptr, Reason::None, true,
                              selfReportJitterVariance(), m_contextSink ? &ctx : nullptr);

    if (m_contextSink && ev.evaluated) {
        ctx.kind = ClaimKind::Cam;
        ctx.author = claim.author;
        ctx.time = now;
        m_contextSink(ctx);
    }
    if (ev.violation) {
        // charged and stopped: no track, no corroboration
        updateScore(scores, ev, 0);
        report(ClaimKind::Cam, claim.author, 0, 0, now, scores, 0, false);
        return;
    }
    admitCam(claim, ego, scores, ev, now);
}

// ----------------------------------------------------------------------------
// CPM: reference pose + perceived objects
// ----------------------------------------------------------------------------

/// The CPM reference pose and the author's own CAMs carry the SAME GNSS fix: it is one station describing
/// one body at two instants, so the positioning bias cancels and only motion, the heading error in the
/// bumper-to-centre shift and the GNSS jitter between the two instants separate them. That is why the pose
/// gate is sub-metre, and it is why the comparison must be intra-source: measured against a multi-source
/// estimate, whose uncertainty floors at the positioning bias, a sub-metre gate fails on honest poses.
double CpTrustEngine::poseCheck(const CpmSnapshot& cpm, const GeoPoint& authorCentre) const
{
    auto cam = m_camHistory.find(cpm.author);
    if (cam == m_camHistory.end() || cam->second.empty()) {
        return -1.0;
    }
    const std::deque<KinematicState>* body = &cam->second;
    // A reference already judged implausible is not used to judge other claims: a CAM trajectory with a falsified
    // speed is caught on its own track, and projecting it would copy its lie onto the CPM's real objects.
    auto scores = m_camScores.find(cpm.author);
    if (scores == m_camScores.end() || !plausible(scores->second)) {
        return -1.0;
    }
    const double dt = cpm.time - body->back().time;
    if (dt < -0.05 || dt > m_params.maxSyncDt) {
        return -1.0;
    }
    const KinematicState projected =
        TemporalTrackSynchronizer::project(*body, std::max(0.0, dt), m_params.syncMode, m_drModel.get());
    const double jitter = m_params.gnss.jitterFraction * m_params.gnss.positionBound;
    const double shift = 0.5 * vehicleLength(cpm.author) * m_params.gnss.headingSigma(projected.speed);
    const double variance = 2.0 * jitter * jitter + shift * shift;
    const double poseGate = SpatialAssociationUtils::computeDynamicGate(projected.speed, std::abs(dt),
                                                                        m_params.association, 0.0, variance);
    const double distance = Geodesy::distance(positionOf(projected), authorCentre);
    return distance / std::max(poseGate, 1e-3);
}

/// The reference pose is the author describing its own body under a signed identity, so the comparison is
/// given rather than searched and a large disagreement has nowhere to escape to. It is graded on the same
/// scale as every other residual and charged once to the author's own claim stream, however many objects
/// the CPM carries. The returned evidence is what the objects of this CPM inherit.
CpTrustEngine::Evidence CpTrustEngine::gradeReferencePose(const CpmSnapshot& cpm, const GeoPoint& authorCentre,
                                                      double now)
{
    Evidence pose;
    const double residual = m_params.enablePoseCheck ? poseCheck(cpm, authorCentre) : -1.0;
    if (residual < 0.0) {
        return pose;
    }
    PhysicalInvarianceResult clean;
    pose = dispatch(clean, residual, 2.0 / m_params.association.kappaGate);
    pose.reason = Reason::PoseConsistency;
    auto authorScores = m_camScores.find(cpm.author);
    if (authorScores != m_camScores.end() && pose.negative > 0.0) {
        updateScore(authorScores->second, pose, 0);
        report(ClaimKind::Cam, cpm.author, 0, 0, now, authorScores->second, 0, false);
    }
    return pose;
}

/// The CAM streams usable as association anchors. A body's anchor is the station's own CAM trajectory:
/// a self-report is the only admissible description of its own body, so no blend is needed and none is
/// made. Streams the caller excluded (the author, the receiver, anything implausible) are left out.
std::vector<ProjectedTrack> CpTrustEngine::projectedBodies(double targetTime,
                                                         const std::set<StationId>& excluded) const
{
    std::vector<ProjectedTrack> projected;
    for (const auto& entry : m_camHistory) {
        if (excluded.count(entry.first) || entry.second.empty()) {
            continue;
        }
        const double dt = targetTime - entry.second.back().time;
        if (dt < -0.05 || dt > m_params.maxSyncDt) {
            continue;
        }
        ProjectedTrack p;
        p.station = entry.first;
        p.state = TemporalTrackSynchronizer::project(entry.second, std::max(0.0, dt), m_params.syncMode,
                                                     m_drModel.get());
        p.syncDt = std::abs(dt);
        projected.push_back(p);
    }
    return projected;
}

/// Check 0 on every object, and alignment to the CPM generation time. Nothing here needs history or
/// identity: it only asks whether a vector could have come from a vehicle and a sensor at all.
void CpTrustEngine::screenObjects(const CpmSnapshot& cpm, const GeoPoint& authorCentre, std::vector<ObjectOutcome>& outcome) const
{
    outcome.assign(cpm.objects.size(), ObjectOutcome{});

    for (size_t j = 0; j < cpm.objects.size(); ++j) {
        const ObjectClaim& o = cpm.objects[j];
        ObjectOutcome& out = outcome[j];
        const bool finite = std::isfinite(o.position.lat) && std::isfinite(o.position.lon) &&
                            std::isfinite(o.speed) && std::isfinite(o.heading);
        const double range = finite ? Geodesy::distance(authorCentre, o.position) : 0.0;
        // PLAUSIBILITY, the single-message bounds, split by which one failed but evaluated in one wrapper
        // so the cost analysis keeps a single timing boundary. A value that cannot be evaluated fails the
        // speed bound vacuously, since without it no bound can be established.
        if (!finite || (m_params.enableAdmissibility && (o.speed < 0.0 || o.speed > m_params.maxSpeed))) {
            out.plausibility = Reason::SpeedPlausibility;
        } else if (m_params.enableAdmissibility &&
                   ((m_params.maxObjectRange > 0.0 && range > m_params.maxObjectRange) ||
                    range < m_params.minSeparation)) {
            out.plausibility = Reason::RangePlausibility;  // outside its author's sensing geometry
        }

        out.aligned = o;  // keeps author and objectId, so nothing downstream needs cpm.objects
        if (o.hasVelocity && finite) {
            out.aligned.position = Geodesy::project(o.position, o.speed, o.heading, cpm.time - o.time);
        }
        out.aligned.time = cpm.time;
        out.aligned.sensorVariance = m_params.radar.positionVariance(range);
    }
}

/// An impossible vector takes the largest penalty and is dropped: it is not kept in its stream's history
/// and never reaches the assignment.
void CpTrustEngine::penaliseImpossibleObjects(const CpmSnapshot& cpm, const std::vector<ObjectOutcome>& outcome, double now)
{
    for (const ObjectOutcome& out : outcome) {
        ObjectStream& t = m_objects[{cpm.author, out.aligned.objectId}];
        t.scores.lastSeen = now;
        if (out.implausible()) {
            const PhysicalInvarianceResult violation = impossible();
            Evidence ev = dispatch(violation, 0.0, 0.0);
            ev.reason = out.plausibility;
            updateScore(t.scores, ev, 0);
            report(ClaimKind::Object, cpm.author, out.aligned.objectId, 0, now, t.scores, 0, false);
        }
    }
}

/// Resolution: which of these anonymous observations are bodies the receiver already knows? Every object
/// that survived check 0 competes in one assignment against the CAM streams still usable as references,
/// plus the receiver itself, which appears in other authors' CPMs but never sends itself a CAM. The
/// assignment returns both the identity and the residual at the argmin, which becomes check 1.
void CpTrustEngine::resolveObjects(const CpmSnapshot& cpm, const EgoView& ego, std::vector<ObjectOutcome>& outcome)
{
    std::vector<ObjectClaim> candidates;
    std::vector<size_t> candidateIndex;  // candidate -> its slot in `outcome`
    for (size_t j = 0; j < outcome.size(); ++j) {
        if (!outcome[j].implausible()) {
            candidates.push_back(outcome[j].aligned);
            candidateIndex.push_back(j);
        }
    }
    if (!m_associationEngine || candidates.empty()) {
        return;
    }

    std::set<StationId> excluded{cpm.author, ego.station};
    for (const auto& entry : m_camScores) {
        if (!plausible(entry.second)) {
            excluded.insert(entry.first);  // an implausible CAM stream is no reference for anyone's objects
        }
    }
    std::vector<ProjectedTrack> tracks = projectedBodies(cpm.time, excluded);

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

    AssociationContext ctx{tracks, candidates, m_params.association};
    for (const AssociationMatch& match : m_associationEngine->associate(ctx)) {
        ObjectOutcome& out = outcome[candidateIndex[match.objectIndex]];
        out.resolvedTo = match.station;
        out.residual = match.normalisedResidual;
    }
}

/**
 * Source-local plausibility: this author's claim about this object, against THIS AUTHOR'S OWN previous
 * claims about the SAME object. Across one author's successive reports of one body its positioning bias
 * is a constant and cancels, exactly as it does between a sender's own CAMs; what remains is the
 * manoeuvre budget plus the independent noise of the two radar measurements. That is a strictly tighter
 * budget than comparing against a multi-observer estimate, where the bias stays in the residual.
 *
 * The association residual is NOT the grade here. Association answers which streams may describe one
 * body, and its outcome is recorded for diagnostics and corroboration; plausibility is a separate
 * statistic measured inside the stream.
 */
std::vector<CpTrustEngine::Evidence> CpTrustEngine::gradeObjects(const CpmSnapshot& cpm, const Evidence& pose,
                                                             std::vector<ObjectOutcome>& outcome, double now)
{
    std::vector<Evidence> evidence(outcome.size());
    for (size_t j = 0; j < outcome.size(); ++j) {
        ObjectOutcome& out = outcome[j];
        if (out.implausible()) {
            continue;
        }
        const ObjectClaim& aligned = out.aligned;
        ObjectStream& t = m_objects[{cpm.author, aligned.objectId}];

        if (pose.violation) {
            // The frame is broken: an object's delivered position is pose + offset, so this claim is wrong
            // as well and inherits the grade the pose earned. Its own residual cannot replace that grade,
            // because a frame error this large displaces the object past its own gate, where it would read
            // as a new object and escape unjudged. The vector is deliberately NOT appended to the stream:
            // a known-wrong frame must not become the reference the next vector is judged against.
            // A pose in the SOFT band is not a violation and does not come here: it is charged once to the
            // author's own CAM stream, where the pose belongs, and every object is still judged on its own
            // merits. Inheriting a soft penalty would both double-charge it across N streams and shield a
            // falsified object from its own check.
            evidence[j] = pose;
        } else if (m_params.enableObjectKinematics) {
            // The variance of the DIFFERENCE between this vector and its predecessor that is not GNSS
            // jitter: the two independent radar measurements. The author's own GNSS error is common to
            // both and cancels.
            const double noiseVariance = t.lastVariance + aligned.sensorVariance;
            ContextSample ctx;
            evidence[j] = check(t.history, stateOf(aligned), nullptr, Reason::None, true, noiseVariance,
                                m_contextSink ? &ctx : nullptr, &m_params.objectKinematics);
            if (m_contextSink && evidence[j].evaluated) {
                ctx.kind = ClaimKind::Object;
                ctx.author = cpm.author;
                ctx.objectId = aligned.objectId;
                ctx.time = now;
                m_contextSink(ctx);
            }
            t.lastVariance = aligned.sensorVariance;
        }

        // Admitted = violated nothing that could be checked. The first vector of a stream has no
        // predecessor, so there is nothing for it to violate: it is admitted and yields no evidence,
        // rather than being charged for its own novelty.
        out.admitted = !evidence[j].violation;
        if (!out.admitted) {
            updateScore(t.scores, evidence[j], 0);
            report(ClaimKind::Object, cpm.author, aligned.objectId, 0, now, t.scores, 0, false);
        }
    }
    return evidence;
}

/// Admission: an object that violated nothing joins its hypothesis, is corroborated by the OTHER stations
/// already on that hypothesis, and is scored once with both kinds of evidence. Nothing is fused: the
/// hypothesis only records who reported what, and the vector becomes its new gating anchor.
void CpTrustEngine::admitObjects(const CpmSnapshot& cpm, const EgoView& ego, std::vector<ObjectOutcome>& outcome,
                               const std::vector<Evidence>& evidence, double now)
{
    for (size_t j = 0; j < outcome.size(); ++j) {
        ObjectOutcome& out = outcome[j];
        if (!out.admitted) {
            continue;
        }
        const ObjectClaim& aligned = out.aligned;
        // Two anonymous observations of one body: two GNSS fixes and two radar measurements.
        const uint64_t track =
            m_ldm.addObject(aligned, out.resolvedTo, ego.station, gate(2.0, 2.0 * aligned.sensorVariance));
        // Keyed by provenance, (author, author-local object id): the stream is what this author claims
        // about this object, independent of which hypothesis the receiver associated it with. A
        // mis-association therefore cannot merge two authors' streams.
        ObjectStream& t = m_objects[{cpm.author, aligned.objectId}];
        // The author is excluded from its own count, so a claim never corroborates itself even though it
        // has just been recorded on the hypothesis. Unique stations are counted, never reports, so one
        // author cannot manufacture corroborators with several local object ids.
        const int peers = track ? m_ldm.independentSources(track, {cpm.author, ego.station},
                                                           now - m_params.inclusionPeriod, now + 0.05)
                                : 0;
        out.peers = peers;
        updateScore(t.scores, evidence[j], peers);
        report(ClaimKind::Object, cpm.author, aligned.objectId, track, now, t.scores, peers, true);
    }
}

std::vector<ObjectOutcome> CpTrustEngine::onCpm(const CpmSnapshot& cpm, const EgoView& ego)
{
    const double now = ego.time;
    housekeeping(now);
    std::vector<ObjectOutcome> outcome;
    if (!m_params.enableCpm) {
        return outcome;  // ablation: this receiver consumes no CPM content at all
    }
    if (cpm.author == ego.station) {
        return outcome;  // the receiver never judges its own message
    }

    const GeoPoint authorCentre = centreFromReference(cpm.referencePosition, cpm.heading, cpm.author);
    const Evidence pose = gradeReferencePose(cpm, authorCentre, now);

    screenObjects(cpm, authorCentre, outcome);
    penaliseImpossibleObjects(cpm, outcome, now);
    resolveObjects(cpm, ego, outcome);
    const std::vector<Evidence> evidence = gradeObjects(cpm, pose, outcome, now);
    admitObjects(cpm, ego, outcome, evidence, now);
    return outcome;
}

void CpTrustEngine::housekeeping(double now)
{
    m_ldm.evict(now);
    for (auto it = m_camScores.begin(); it != m_camScores.end();) {
        if (now - it->second.lastSeen > m_params.trackTimeout) {
            m_lengths.erase(it->first);
            m_camHistory.erase(it->first);
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
