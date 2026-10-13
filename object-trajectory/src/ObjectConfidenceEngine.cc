#include "../include/ObjectConfidenceEngine.h"
#include "../include/Hungarian.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ot {

/// A motion vector as the filter consumes it. The heading stays in wire convention here --
/// clockwise from north -- and the filter converts it, because the filter is the only thing that
/// knows its own angular frame.
Measurement ObjectConfidenceEngine::measurementOf(const MotionVector& v)
{
    Measurement z;
    z.px = v.x;
    z.py = v.y;
    z.speed = v.speed;
    z.heading = v.heading;
    return z;
}

/// Stage 1. One vector against an absolute bound. No history, so this is the cheapest filter and
/// it runs first: nonsense should never reach the O(N) work below.
std::vector<const MotionVector*>
ObjectConfidenceEngine::plausibilityChecks(const std::vector<MotionVector>& in) const
{
    std::vector<const MotionVector*> out;
    out.reserve(in.size());
    for (const MotionVector& v : in) {
        const bool finite = std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.speed) &&
                            std::isfinite(v.heading);
        if (!finite || v.speed < 0.0 || v.speed > m_params.maxSpeed) {
            continue;
        }
        out.push_back(&v);
    }
    return out;
}

/// Stage 2. Every track moves to `now`, once for the whole message. Doing this per vector would
void ObjectConfidenceEngine::projectKnownObjects(double now)
{
    if (m_lastProjection < 0.0) {
        m_lastProjection = now;
        return;
    }
    const double dt = now - m_lastProjection;
    if (dt <= 0.0) {
        return;
    }
    for (ObjectTrack& t : m_tracks) {
        t.kf.predict(dt);
    }
    m_lastProjection = now;
}

/// Stage 3. Gate FIRST, then assign jointly over what survived.
///
/// Gating before the assignment, rather than after, matters: an out-of-gate pair left in the matrix
/// can win a column and pull a track away from the measurement that actually needed it. Forbidden
/// cells are left at Hungarian::kForbidden so the solver never prefers them.
///
/// Two exact optimisations live here. Neither changes which pairs are gated or which assignment is
/// chosen -- they only avoid work that provably cannot matter. Both were added because the cost
/// evaluator showed the unoptimised version missing a 100 ms budget at N = 400, m = 20.
///
///   COARSE GATE. The ellipsoidal test needs a 4x4 inversion per pair, which is O(N*m) inversions
///   per message. But if y'S^-1 y <= g then for ANY unit vector e, (e'y)^2 <= g * e'Se. Taking e as
///   each axis in turn gives four necessary conditions -- dx^2 <= g*S00 and so on -- that cost two
///   multiplies and reuse S's diagonal, which depends only on the track. Failing any of them means
///   the pair cannot be inside the ellipsoid, so the inversion is skipped. Conservative by
///   construction: it can never reject a pair the full test would have accepted. This is the
///   standard coarse-then-fine gate of the tracking literature.
///
///   COMPACT MATRIX. The solver this replaced padded to max(rows, cols) and ran in cubic time in
///   that side, so passing all
///   N tracks as rows costs O(N^3) even when a message carries three objects. A track with no
///   in-gate candidate cannot legitimately be assigned to anything, so only tracks with at least
///   one gated pair become rows. The solve then scales with what is actually contested, not with
///   the size of the neighbourhood.
ObjectConfidenceEngine::Assignment
ObjectConfidenceEngine::associate(const std::vector<const MotionVector*>& v) const
{
    Assignment a;
    a.trackForVector.assign(v.size(), Hungarian::kNone);
    a.cost.assign(v.size(), -1.0);
    a.runnerUp.assign(v.size(), -1.0);
    a.gated.assign(v.size(), 0);
    if (v.empty() || m_tracks.empty()) {
        return a;
    }

    std::vector<double> best(v.size(), std::numeric_limits<double>::infinity());
    std::vector<double> second(v.size(), std::numeric_limits<double>::infinity());

    // every gated pair, found before anything is laid out as a matrix
    struct GatedPair {
        std::size_t track;
        std::size_t vec;
        double d;
    };
    std::vector<GatedPair> pairs;
    pairs.reserve(v.size() * 4);

    // compact row indices: track -> row, and back
    std::vector<std::size_t> rowOfTrack(m_tracks.size(), Hungarian::kNone);
    std::vector<std::size_t> trackOfRow;
    trackOfRow.reserve(std::min(m_tracks.size(), v.size() * 4));

    for (std::size_t r = 0; r < m_tracks.size(); ++r) {
        const ObjectTrack& t = m_tracks[r];

        // S's diagonal depends only on the track, so the coarse bounds are computed once per track
        // rather than once per pair. sqrt(gate * S_ii) is the widest this residual can ever be.
        const double limPos = std::sqrt(m_params.gate * (t.kf.covariance(kCtraPx, kCtraPx) + m_params.noise.px));
        const double limPosY = std::sqrt(m_params.gate * (t.kf.covariance(kCtraPy, kCtraPy) + m_params.noise.py));
        const double limSpeed = std::sqrt(m_params.gate * (t.kf.covariance(kCtraSpeed, kCtraSpeed) + m_params.noise.speed));
        const double limHead = std::sqrt(m_params.gate * (t.kf.covariance(kCtraTheta, kCtraTheta) + m_params.noise.heading));

        for (std::size_t c = 0; c < v.size(); ++c) {
            const MotionVector& mv = *v[c];
            if (m_params.classMustMatch && t.objectClass != 0 && mv.objectClass != 0 &&
                t.objectClass != mv.objectClass) {
                continue;
            }

            // coarse gate: four cheap necessary conditions, position first because it is the most
            // selective. Any failure means the ellipsoidal test would fail too.
            if (m_params.coarseGate &&
                (std::abs(mv.x - t.kf.state(kCtraPx)) > limPos ||
                 std::abs(mv.y - t.kf.state(kCtraPy)) > limPosY ||
                 std::abs(mv.speed - t.kf.state(kCtraSpeed)) > limSpeed ||
                 std::abs(wrapPi(lab::thetaFromHeading(mv.heading) - t.kf.state(kCtraTheta))) > limHead)) {
                continue;
            }

            const Innovation y = t.kf.innovationAgainst(measurementOf(mv));
            const double d = mahalanobis(y, t.kf.innovationCovariance(m_params.noise));
            if (d < 0.0 || d > m_params.gate) {
                continue;   // outside the 95 % ellipsoid: not a candidate at all
            }

            if (rowOfTrack[r] == Hungarian::kNone) {
                rowOfTrack[r] = trackOfRow.size();
                trackOfRow.push_back(r);
            }
            pairs.push_back(GatedPair{r, c, d});
            ++a.gated[c];
            if (d < best[c]) {
                second[c] = best[c];
                best[c] = d;
            } else if (d < second[c]) {
                second[c] = d;
            }
        }
    }

    if (trackOfRow.empty()) {
        return a;   // nothing gated anywhere: every vector starts its own trajectory
    }

    Hungarian::CostMatrix m;
    m.resize(trackOfRow.size(), v.size());
    for (const GatedPair& p : pairs) {
        m.set(rowOfTrack[p.track], p.vec, p.d);
    }

    const std::vector<std::size_t> columnToRow = Hungarian::solveAssignment(m);

    for (std::size_t c = 0; c < v.size(); ++c) {
        const std::size_t row = columnToRow[c];
        if (row == Hungarian::kNone || m.at(row, c) >= Hungarian::kForbidden) {
            continue;   // padded or forbidden cell: this vector matched nothing
        }
        a.trackForVector[c] = trackOfRow[row];
        a.cost[c] = m.at(row, c);
        a.runnerUp[c] = std::isfinite(second[c]) ? second[c] : -1.0;
    }
    return a;
}

/// Stage 4b. Fold the report in. Predict already happened in stage 2 -- do not predict again here,
/// or the covariance inflates and every gate silently widens.
bool ObjectConfidenceEngine::kalmanUpdate(ObjectTrack& t, const MotionVector& v)
{
    return t.kf.update(measurementOf(v), m_params.noise);
}

/// Stage 5. Whatever matched nothing becomes a new trajectory, starting uncertain.
void ObjectConfidenceEngine::initiateTrack(const MotionVector& v, double now)
{
    ObjectTrack t;
    t.id = m_nextId++;
    t.kf.state(kCtraPx) = v.x;
    t.kf.state(kCtraPy) = v.y;
    t.kf.state(kCtraSpeed) = v.speed;
    // The filter works in the laboratory model's angular frame, so convert here and nowhere else.
    t.kf.state(kCtraTheta) = lab::thetaFromHeading(v.heading);
    t.kf.state(kCtraYawRate) = v.hasYawRate ? v.yawRate : 0.0;
    // Nothing reports acceleration, so a new track assumes none and lets the filter find it.
    t.kf.state(kCtraAccel) = 0.0;

    // A new track believes nothing is correlated yet: the off-diagonal terms are born later, when
    // the prediction mixes the states together.
    t.kf.covariance = CtraCovariance::Zero();
    t.kf.covariance(kCtraPx, kCtraPx) = m_params.initialPosVar;
    t.kf.covariance(kCtraPy, kCtraPy) = m_params.initialPosVar;
    t.kf.covariance(kCtraSpeed, kCtraSpeed) = m_params.initialVelVar;
    t.kf.covariance(kCtraTheta, kCtraTheta) = m_params.initialHeadVar;
    t.kf.covariance(kCtraYawRate, kCtraYawRate) = m_params.initialRateVar;
    t.kf.covariance(kCtraAccel, kCtraAccel) = m_params.initialAccelVar;
    t.firstSeen = now;
    t.lastUpdate = now;
    t.hits = 1;
    t.objectClass = v.objectClass;
    t.length = v.length;
    t.width = v.width;
    t.contributors[v.author] = now;
    t.trueStationHits[v.trueStation] = 1;
    m_tracks.push_back(t);
}

/// Stage 6. A track nobody has fed for a while is gone. Without this the store grows without bound
/// and N -- the term the predict stage scales with -- grows with it.
void ObjectConfidenceEngine::retireStaleTracks(double now)
{
    m_tracks.erase(std::remove_if(m_tracks.begin(), m_tracks.end(),
                                  [&](const ObjectTrack& t) {
                                      return now - t.lastUpdate > m_params.retireAfter;
                                  }),
                   m_tracks.end());
}

/**
 * The default policy, deliberately simple: every fused report splits one unit of evidence between
 * "this trajectory is real and moving as believed" and "it is not", according to how well the
 * report agreed. The published confidence is alpha / (alpha + beta).
 *
 * The split is the important part. A report that lands in the middle of the gate is strong evidence
 * FOR; a report that barely squeezed inside the gate is mostly evidence AGAINST, even though it was
 * accepted and fused. Treating a weak agreement as merely "no evidence" is what makes a score
 * freeze: confidence then only ever rises, and a trajectory being fed consistently bad reports
 * looks exactly like one being fed good ones.
 *
 * This is a placeholder with the right shape, not the final answer -- that is what the seam is for.
 */
double ObjectConfidenceEngine::defaultConfidence(ObjectTrack& t, 
                                                 const MotionVector&,
                                                 const VectorRecord& r) const
{
    switch (r.outcome) {
        case Outcome::Fused: {
            // normalisedGap is the squared Mahalanobis distance; at the gate it is worthless,
            // at 0 it is perfect. Everything inside the gate therefore maps to [0, 1].
            const double agreement = 1.0 - std::min(1.0, r.normalisedGap / m_params.gate);

            // How much this one report is allowed to say. A contested match -- the runner-up track
            // nearly won it -- is weak testimony about EITHER track, so it moves the score less.
            double weight = 1.0;
            if (r.runnerUpGap >= 0.0) {
                const double margin = (r.runnerUpGap - r.normalisedGap) / m_params.gate;
                weight = std::min(1.0, std::max(0.1, margin));
            }
            // A second independent station agreeing is worth more than the same one again: it is
            // the one thing here that no single sender can manufacture alone.
            if (r.contributors > 1) {
                weight = std::min(1.5, weight * 1.5);
            }

            t.alpha += weight * agreement;
            t.beta += weight * (1.0 - agreement);
            break;
        }
        case Outcome::Rejected:
            // matched a track and then failed to fuse: unambiguously evidence against
            t.beta += 1.0;
            break;
        case Outcome::StartedTrack:
        case Outcome::Dropped:
            break;   // nothing was learned about an existing trajectory
    }
    return t.alpha / (t.alpha + t.beta);
}

std::vector<VectorRecord>
ObjectConfidenceEngine::onMotionVectors(const std::vector<MotionVector>& vectors,
                                        uint64_t receiver,
                                        double now)
{
    std::vector<VectorRecord> out;
    out.reserve(vectors.size());
    MaybeStage whole(m_timer, "onMotionVectors");

    std::vector<const MotionVector*> kept;
    {
        MaybeStage s(m_timer, "1 plausibility O(m)");
        kept = plausibilityChecks(vectors);                                      // 1
    }

    // removes the motion vectors that didnt passed the plausibility checks
    for (const MotionVector& v : vectors) {
        if (std::find(kept.begin(), kept.end(), &v) == kept.end()) {
            VectorRecord r;
            r.t = v.t;
            r.receiver = receiver;
            r.author = v.author;
            r.origin = v.origin;
            r.trueStation = v.trueStation;
            r.outcome = Outcome::Dropped;
            out.push_back(r);
        }
    }

    {
        MaybeStage s(m_timer, "2 project O(N)");
        projectKnownObjects(now);                                                // 2
    }
    Assignment a;
    {
        MaybeStage s(m_timer, "3 associate O(N*m)");
        a = associate(kept);                                                     // 3
    }
    {
    MaybeStage rest(m_timer, "4-5 fuse + initiate");
    for (std::size_t c = 0; c < kept.size(); ++c) {
        const MotionVector& v = *kept[c];
        VectorRecord r;
        r.t = v.t;
        r.receiver = receiver;
        r.author = v.author;
        r.origin = v.origin;
        r.trueStation = v.trueStation;
        r.gatedCandidates = a.gated[c];
        r.runnerUpGap = a.runnerUp[c];

        if (a.trackForVector[c] == Hungarian::kNone) {
            initiateTrack(v, now);                                               // 5
            r.outcome = Outcome::StartedTrack;
            r.trackId = m_tracks.back().id;
            r.contributors = 1;
            r.confidence = m_tracks.back().confidence;
            out.push_back(r);
            continue;
        }

        ObjectTrack& t = m_tracks[a.trackForVector[c]];
        r.trackId = t.id;
        r.normalisedGap = a.cost[c];
        r.gap = std::hypot(v.x - t.x(), v.y - t.y());

        // stage 4: the gate already decided this is consistent, so fuse
        const bool fused = kalmanUpdate(t, v);                                   // 4
        r.outcome = fused ? Outcome::Fused : Outcome::Rejected;
        if (fused) {
            t.hits += 1;
            t.misses = 0;
            t.lastUpdate = now;
            t.contributors[v.author] = now;
            t.trueStationHits[v.trueStation] += 1;
            if (!t.confirmed && t.hits >= m_params.confirmAfter) {
                t.confirmed = true;
            }
        }
        r.contributors = static_cast<int>(t.contributors.size());

        t.confidence = m_confidence ? m_confidence(t, v, r) : defaultConfidence(t, v, r);
        r.confidence = t.confidence;
        out.push_back(r);
    }
    }

    {
        MaybeStage s(m_timer, "6 retire O(N)");
        retireStaleTracks(now);                                                  // 6
    }
    return out;
}

}  // namespace ot
