#ifndef WORLD_MAP_H_
#define WORLD_MAP_H_

#include "CpmTypes.h"
#include "Geodesy.h"
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace artery {

/// Who made a statement about a track.
enum class SourceKind { Ego, Cam, Cpm };

/// When and from where an author sent a CPM (where its radars looked).
struct CpmRecord {
    StationId author = 0;
    double time = 0.0;
    GeoPoint centre;
    double heading = 0.0;
};

/// A CPM object statement indexed by its author (what the author said, wherever it landed).
struct LdmAuthorEntry {
    uint64_t track = 0;
    ObjectClaim claim;
};

/// One statement about a track, kept with its source (provenance). Nothing is fused away.
struct LdmContribution {
    StationId station = 0;   ///< Ego: the receiver; Cam: the sender (the body itself); Cpm: the author
    SourceKind kind = SourceKind::Cpm;
    uint32_t localId = 0;    ///< Ego: LEM tracking id; Cpm: CPM object id; Cam: 0
    ObjectClaim claim;       ///< time, position (centre), velocity
};

/// One physical object the receiver believes may exist.
struct LdmTrack {
    uint64_t id = 0;
    StationId identity = -1;  ///< the station whose CAM describes this body, -1 if anonymous
    std::deque<LdmContribution> contributions;  ///< time-ordered, all sources, last `retention` seconds
    LdmContribution latest;                     ///< most recent statement (any source)
    std::map<std::pair<int, StationId>, double> lastBy;  ///< (source kind, station) -> time of its latest statement
    double lastUpdate = -1.0;
};

/**
 * @brief Local Dynamic Map (LDM) of one receiver: one track per physical object, with provenance.
 *
 * Every CAM (the sender's body), every admissible CPM object and every ego radar detection is attached to a
 * track: first by continuity (the same source kept the same local id or station), then by the nearest track
 * inside the gate, else a new track. Contributions keep their source, so a claim can be compared with what
 * OTHER sources say about the same physical object (leave-one-out).
 */
class WorldMap {
public:
    static constexpr uint64_t kFirstTrackId = uint64_t(1) << 20;  ///< above every CPM object id

    explicit WorldMap(double retention = 4.0) : m_retention(retention) {}

    /// The CAM sender's own body (centre).
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
        append(id, LdmContribution{station, SourceKind::Cam, 0, c});
        return id;
    }

    /**
     * An admissible CPM object (aligned to its CPM time). `resolvedTo` is the CAM station it was resolved to
     * (-1 if unresolved). Returns the track, or 0 if the object is the receiver itself (`receiver`).
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
        const auto key = std::make_tuple(static_cast<int>(SourceKind::Cpm), claim.author, static_cast<uint32_t>(claim.objectId));
        if (id == 0) {
            id = continuing(key, claim.position, claim.time, gate);
        }
        if (id == 0) {
            id = nearest(claim.position, claim.time, gate, claim.author, SourceKind::Cpm, false);
        }
        if (id == 0) {
            id = create();
        }
        m_continuity[key] = id;
        append(id, LdmContribution{claim.author, SourceKind::Cpm, claim.objectId, claim});
        return id;
    }

    /// A detection of the receiver's own radar at `time`.
    uint64_t addEgo(StationId receiver, const EgoDetection& d, double time, double gate)
    {
        ObjectClaim c;
        c.author = receiver;
        c.time = time;
        c.position = d.position;
        c.speed = d.speed;
        c.heading = d.heading;
        c.hasVelocity = d.hasVelocity;
        const auto key = std::make_tuple(static_cast<int>(SourceKind::Ego), receiver, d.id);
        uint64_t id = continuing(key, d.position, time, gate);
        if (id == 0) {
            id = nearest(d.position, time, gate, receiver, SourceKind::Ego, true);
        }
        if (id == 0) {
            id = create();
        }
        m_continuity[key] = id;
        append(id, LdmContribution{receiver, SourceKind::Ego, d.id, c});
        return id;
    }

    const LdmTrack* track(uint64_t id) const
    {
        auto it = m_tracks.find(id);
        return it == m_tracks.end() ? nullptr : &it->second;
    }

    /**
     * Number of distinct stations, other than `excluded`, whose CAM or CPM put a statement on the track during
     * [t0, t1]: the independent confirmations of a claim about this object. The receiver's own radar is not
     * counted here (it is the Observation check).
     */
    int independentSources(uint64_t id, const std::set<StationId>& excluded, double t0, double t1) const
    {
        const LdmTrack* t = track(id);
        if (!t) {
            return 0;
        }
        std::set<StationId> sources;
        for (const LdmContribution& c : t->contributions) {
            if (c.kind != SourceKind::Ego && !excluded.count(c.station) && c.claim.time >= t0 && c.claim.time <= t1) {
                sources.insert(c.station);
            }
        }
        return static_cast<int>(sources.size());
    }

    /// Where and when an author sent a CPM (what its radars covered).
    void addCpm(const CpmRecord& record) { m_cpms[record.author].push_back(record); }

    /// Authors with a CPM in the retention window.
    std::vector<StationId> authors() const
    {
        std::vector<StationId> ids;
        for (const auto& entry : m_cpms) {
            if (!entry.second.empty()) {
                ids.push_back(entry.first);
            }
        }
        return ids;
    }

    /// Calls f(const CpmRecord&) for every CPM `author` sent during [t0, t1].
    template <typename F>
    void forEachCpm(StationId author, double t0, double t1, F&& f) const
    {
        auto it = m_cpms.find(author);
        if (it == m_cpms.end()) {
            return;
        }
        for (const CpmRecord& r : it->second) {
            if (r.time >= t0 && r.time <= t1) {
                f(r);
            }
        }
    }

    /**
     * Did `author` report the object during [t0, t1]: a statement on `track`, or any statement within `gate` of p
     * (p known at `time`, projected with its speed and heading to each statement's time)? The second test is a safety
     * net for association misses: silence is only silence if the author said nothing near the spot.
     */
    bool reported(StationId author, uint64_t track, const GeoPoint& p, double time, double speed, double heading,
                  double t0, double t1, double gate) const
    {
        auto it = m_byAuthor.find(author);
        if (it == m_byAuthor.end()) {
            return false;
        }
        for (const LdmAuthorEntry& e : it->second) {
            if (e.claim.time < t0 || e.claim.time > t1) {
                continue;
            }
            if (track && e.track == track) {
                return true;
            }
            const GeoPoint aligned = Geodesy::project(p, speed, heading, e.claim.time - time);
            if (Geodesy::distance(aligned, e.claim.position) <= gate) {
                return true;
            }
        }
        return false;
    }

    /// Calls f(trackId, position at `time`) for every track with a statement within `window` of `time`.
    template <typename F>
    void forEachTrackAt(double time, double window, F&& f) const
    {
        for (const auto& entry : m_tracks) {
            const LdmContribution& latest = entry.second.latest;
            if (std::abs(latest.claim.time - time) <= window) {
                f(entry.first, positionAt(latest, time));
            }
        }
    }

    size_t size() const { return m_tracks.size(); }

    void evict(double now)
    {
        for (auto it = m_tracks.begin(); it != m_tracks.end();) {
            auto& list = it->second.contributions;
            while (!list.empty() && now - list.front().claim.time > m_retention) {
                list.pop_front();
            }
            it = list.empty() ? m_tracks.erase(it) : std::next(it);
        }
        for (auto it = m_continuity.begin(); it != m_continuity.end();) {
            it = m_tracks.count(it->second) ? std::next(it) : m_continuity.erase(it);
        }
        for (auto it = m_bodies.begin(); it != m_bodies.end();) {
            it = m_tracks.count(it->second) ? std::next(it) : m_bodies.erase(it);
        }
        for (auto it = m_byAuthor.begin(); it != m_byAuthor.end();) {
            auto& queue = it->second;
            while (!queue.empty() && now - queue.front().claim.time > m_retention) {
                queue.pop_front();
            }
            it = queue.empty() ? m_byAuthor.erase(it) : std::next(it);
        }
        for (auto it = m_cpms.begin(); it != m_cpms.end();) {
            auto& queue = it->second;
            while (!queue.empty() && now - queue.front().time > m_retention) {
                queue.pop_front();
            }
            it = queue.empty() ? m_cpms.erase(it) : std::next(it);
        }
    }

private:
    using ContinuityKey = std::tuple<int, StationId, uint32_t>;

    static GeoPoint positionAt(const LdmContribution& c, double time)
    {
        return c.claim.hasVelocity ? Geodesy::project(c.claim.position, c.claim.speed, c.claim.heading, time - c.claim.time)
                                   : c.claim.position;
    }

    uint64_t create()
    {
        const uint64_t id = kFirstTrackId + m_next++;
        m_tracks[id].id = id;
        return id;
    }

    void append(uint64_t id, const LdmContribution& c)
    {
        LdmTrack& t = m_tracks[id];
        t.contributions.push_back(c);
        if (c.claim.time >= t.lastUpdate) {
            t.latest = c;
            t.lastUpdate = c.claim.time;
        }
        t.lastBy[{static_cast<int>(c.kind), c.station}] = c.claim.time;
        if (c.kind == SourceKind::Cpm) {
            m_byAuthor[c.station].push_back(LdmAuthorEntry{id, c.claim});
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
        return Geodesy::distance(positionAt(t->second.latest, time), p) <= gate ? it->second : 0;
    }

    /// Nearest track within the gate at `time`, excluding tracks the same source already feeds with another
    /// statement in the last 0.5 s (one source does not report one object twice). CAM bodies are candidates only
    /// for ego detections: CPM objects join a body through resolution against the CAM track.
    uint64_t nearest(const GeoPoint& p, double time, double gate, StationId station, SourceKind kind, bool bodies) const
    {
        uint64_t best = 0;
        double bestDist = gate;
        for (const auto& entry : m_tracks) {
            const LdmTrack& t = entry.second;
            if (!bodies && t.identity >= 0) {
                continue;
            }
            auto last = t.lastBy.find({static_cast<int>(kind), station});
            if (last != t.lastBy.end() && std::abs(last->second - time) < 0.5) {
                continue;  // busy: the same source already feeds this track with another statement
            }
            if (std::abs(t.latest.claim.time - time) > m_retention) {
                continue;
            }
            const double d = Geodesy::distance(positionAt(t.latest, time), p);
            if (d <= bestDist) {
                bestDist = d;
                best = entry.first;
            }
        }
        return best;
    }

    double m_retention;
    uint64_t m_next = 1;
    std::map<uint64_t, LdmTrack> m_tracks;
    std::map<ContinuityKey, uint64_t> m_continuity;
    std::map<StationId, uint64_t> m_bodies;
    std::map<StationId, std::deque<CpmRecord>> m_cpms;
    std::map<StationId, std::deque<LdmAuthorEntry>> m_byAuthor;
};

}  // namespace artery

#endif  // WORLD_MAP_H_
