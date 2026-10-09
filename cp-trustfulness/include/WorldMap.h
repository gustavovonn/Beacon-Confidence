#ifndef WORLD_MAP_H_
#define WORLD_MAP_H_

#include "CpmTypes.h"
#include "Geodesy.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace artery {

/// One admitted motion vector, with the station that produced it.
struct LdmVector {
    StationId station = 0;  ///< the CAM sender, or the CPM author
    ObjectClaim claim;      ///< time, position (centre), velocity
};

/**
 * One object hypothesis: the receiver's guess that a set of claim streams are describing the same body.
 *
 * This is an ASSOCIATION CACHE ENTRY, not an estimate. `reference` is the latest admitted vector, kept
 * only so the next vector has something to be gated against; nothing is blended into it and no residual
 * is ever measured against it. Plausibility is judged by each claim stream against its OWN history, where
 * the author's positioning bias cancels; the hypothesis exists only to answer which streams may be
 * talking about the same body, and therefore who corroborates whom.
 *
 * The hypothesis carries no confidence. Confidence lives on the claim streams, outside this map.
 */
struct LdmTrack {
    StationId identity = -1;               ///< the station whose CAM describes this body, -1 if anonymous
    LdmVector reference;                   ///< the latest admitted vector: what new vectors are gated against
    std::map<StationId, double> admitted;  ///< station -> time of its last admitted vector; the contributor set
    double lastAdmission = -1.0;
};

/**
 * @brief Receiver-local association cache: one hypothesis per apparent physical object.
 *
 * A hypothesis stores only what association and corroboration need: the latest admitted vector and the
 * time each station last fed it. It is dropped when nobody feeds it for `lifetime` seconds.
 *
 * Deliberately absent: state fusion, a fused trajectory, a per-hypothesis variance and a per-hypothesis
 * score. Fusion was measured to break the intra-source comparisons that plausibility depends on, and is
 * deferred to a separate study (`docs/fusion_study_issue.md`).
 */
class WorldMap {
public:
    static constexpr uint64_t kFirstTrackId = uint64_t(1) << 20;  ///< above every CPM object id

    explicit WorldMap(double lifetime = 1.1) : m_lifetime(lifetime) {}

    /// An admitted CAM vector: the sender's own body. A self-report is the only admissible anchor for its
    /// own body, so it always becomes the reference.
    uint64_t addCam(StationId station, const KinematicState& centre)
    {
        ObjectClaim c;
        c.author = station;
        c.time = centre.time;
        c.position = GeoPoint{centre.lat, centre.lon};
        c.speed = centre.speed;
        c.heading = centre.heading;
        c.hasVelocity = true;
        uint64_t id = 0;
        auto body = m_bodies.find(station);
        if (body != m_bodies.end() && m_tracks.count(body->second)) {
            id = body->second;
        } else {
            id = create();
            m_tracks[id].identity = station;
            m_bodies[station] = id;
        }
        admit(id, LdmVector{station, c});
        return id;
    }

    /**
     * An admitted CPM object vector (aligned to its CPM time). `resolvedTo` is the CAM station it was
     * resolved to (-1 if unresolved). Returns the hypothesis, or 0 if the object is the receiver itself.
     */
    uint64_t addObject(const ObjectClaim& claim, StationId resolvedTo, StationId receiver, double gate)
    {
        if (resolvedTo >= 0 && resolvedTo == receiver) {
            return 0;
        }
        uint64_t id = 0;
        if (resolvedTo >= 0) {
            auto body = m_bodies.find(resolvedTo);
            if (body != m_bodies.end() && m_tracks.count(body->second)) {
                id = body->second;
            }
        }
        const ContinuityKey key{claim.author, static_cast<uint32_t>(claim.objectId)};
        if (id == 0) {
            id = continuing(key, claim.position, claim.time, gate);
        }
        if (id == 0) {
            id = nearest(claim.position, claim.time, gate, claim.author);
        }
        if (id == 0) {
            id = create();
        }
        m_continuity[key] = id;
        admit(id, LdmVector{claim.author, claim});
        return id;
    }

    const LdmTrack* track(uint64_t id) const
    {
        auto it = m_tracks.find(id);
        return it == m_tracks.end() ? nullptr : &it->second;
    }



    /// Number of distinct stations, other than `excluded`, with an admitted vector on the hypothesis
    /// during [t0, t1]. This is the corroboration count: unique stations, never reports.
    int independentSources(uint64_t id, const std::set<StationId>& excluded, double t0, double t1) const
    {
        const LdmTrack* t = track(id);
        if (!t) {
            return 0;
        }
        int n = 0;
        for (const auto& entry : t->admitted) {
            n += !excluded.count(entry.first) && entry.second >= t0 && entry.second <= t1;
        }
        return n;
    }

    size_t size() const { return m_tracks.size(); }

    void evict(double now)
    {
        for (auto it = m_tracks.begin(); it != m_tracks.end();) {
            it = (now - it->second.lastAdmission > m_lifetime) ? m_tracks.erase(it) : std::next(it);
        }
        for (auto it = m_continuity.begin(); it != m_continuity.end();) {
            it = m_tracks.count(it->second) ? std::next(it) : m_continuity.erase(it);
        }
        for (auto it = m_bodies.begin(); it != m_bodies.end();) {
            it = m_tracks.count(it->second) ? std::next(it) : m_bodies.erase(it);
        }
    }

private:
    using ContinuityKey = std::pair<StationId, uint32_t>;

    static GeoPoint positionAt(const LdmVector& v, double time)
    {
        return v.claim.hasVelocity ? Geodesy::project(v.claim.position, v.claim.speed, v.claim.heading, time - v.claim.time)
                                   : v.claim.position;
    }

    uint64_t create()
    {
        const uint64_t id = kFirstTrackId + m_next++;
        m_tracks[id];  // default-construct the hypothesis
        return id;
    }

    /**
     * Record an admitted vector. The contribution is always recorded, because a late vector is still
     * evidence that its author saw the object; but a vector OLDER than the current reference does not move
     * the anchor, since the anchor's only job is to gate what comes next. Blending is deliberately absent.
     */
    void admit(uint64_t id, const LdmVector& v)
    {
        LdmTrack& t = m_tracks[id];
        const double time = v.claim.time;
        const bool first = t.lastAdmission < 0.0;
        t.admitted[v.station] = std::max(t.admitted[v.station], time);
        t.lastAdmission = std::max(t.lastAdmission, time);
        if (first || time >= t.reference.claim.time) {
            t.reference = v;
        }
    }

    uint64_t continuing(const ContinuityKey& key, const GeoPoint& p, double time, double gate) const
    {
        auto it = m_continuity.find(key);
        if (it == m_continuity.end()) {
            return 0;
        }
        auto t = m_tracks.find(it->second);
        if (t == m_tracks.end()) {
            return 0;
        }
        return Geodesy::distance(positionAt(t->second.reference, time), p) <= gate ? it->second : 0;
    }

    /// Anonymous hypothesis with the nearest reference within the gate, not fed by `author` in the last
    /// 0.5 s (one author does not report one object twice). CAM bodies are no candidates: objects join a
    /// body through resolution, not through proximity.
    uint64_t nearest(const GeoPoint& p, double time, double gate, StationId author) const
    {
        uint64_t best = 0;
        double bestDist = gate;
        for (const auto& entry : m_tracks) {
            const LdmTrack& t = entry.second;
            if (t.identity >= 0) {
                continue;
            }
            auto last = t.admitted.find(author);
            if (last != t.admitted.end() && std::abs(last->second - time) < 0.5) {
                continue;
            }
            const double d = Geodesy::distance(positionAt(t.reference, time), p);
            if (d <= bestDist) {
                bestDist = d;
                best = entry.first;
            }
        }
        return best;
    }

    double m_lifetime;
    uint64_t m_next = 1;
    std::map<uint64_t, LdmTrack> m_tracks;
    std::map<ContinuityKey, uint64_t> m_continuity;
    std::map<StationId, uint64_t> m_bodies;
};

}  // namespace artery

#endif  // WORLD_MAP_H_
