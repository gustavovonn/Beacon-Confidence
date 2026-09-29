// Unit tests for the OMNeT-free confidence core.
// Build: see cp-trustfulness/tests/run_tests.sh
#include "../include/AssociationEngines.h"
#include "../include/CheckerPipeline.h"
#include "../include/DeadReckoningModels.h"
#include "../include/ErrorModels.h"
#include "../include/Geodesy.h"
#include "../include/SensorCoverage.h"
#include "../include/TemporalTrackSynchronizer.h"
#include "../include/TrustEngine.h"
#include "../include/TrustEvaluators.h"
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace artery;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                                   \
    do {                                                                              \
        ++g_checks;                                                                   \
        if (!(cond)) {                                                                \
            ++g_failures;                                                             \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
        }                                                                             \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        ++g_checks;                                                                                    \
        const double va = (a), vb = (b);                                                               \
        if (!(std::abs(va - vb) <= (tol))) {                                                           \
            ++g_failures;                                                                              \
            std::printf("  FAIL %s:%d: %s = %.6f, expected %.6f +- %g\n", __FILE__, __LINE__, #a, va, vb, \
                        static_cast<double>(tol));                                                     \
        }                                                                                              \
    } while (0)

const GeoPoint kOrigin{49.6, 6.1};  // Luxembourg

GeoPoint at(double east, double north)
{
    return Geodesy::fromLocal(kOrigin, LocalPoint{east, north});
}

// ── Helpers ──

KinematicState state(const GeoPoint& p, double speed, double heading, double time)
{
    KinematicState s;
    s.lat = p.lat;
    s.lon = p.lon;
    s.speed = speed;
    s.heading = heading;
    s.time = time;
    return s;
}

ObjectClaim object(StationId author, uint16_t id, const GeoPoint& p, double speed, double heading, double time,
                   bool hasVelocity = true)
{
    ObjectClaim o;
    o.author = author;
    o.objectId = id;
    o.position = p;
    o.speed = speed;
    o.heading = heading;
    o.time = time;
    o.hasVelocity = hasVelocity;
    return o;
}

void testGeodesy()
{
    const GeoPoint p = at(30.0, -40.0);
    const LocalPoint back = Geodesy::toLocal(kOrigin, p);
    CHECK_NEAR(back.east, 30.0, 1e-6);
    CHECK_NEAR(back.north, -40.0, 1e-6);
    CHECK_NEAR(Geodesy::distance(kOrigin, p), 50.0, 0.05);

    // Heading 90 (east): forward is +east, right is -north (south)
    const LocalPoint o = Geodesy::toLocal(kOrigin, Geodesy::offset(kOrigin, 90.0, 10.0, 2.0));
    CHECK_NEAR(o.east, 10.0, 1e-6);
    CHECK_NEAR(o.north, -2.0, 1e-6);

    CHECK_NEAR(Geodesy::bearing(LocalPoint{1.0, 0.0}), 90.0, 1e-9);
    CHECK_NEAR(Geodesy::wrap180(270.0), -90.0, 1e-9);
}

void testGridFrame()
{
    GridFrame aligned;  // grid == true
    LocalPoint d = aligned.fromArteryDelta(10.0, 20.0);
    CHECK_NEAR(d.east, 10.0, 1e-9);
    CHECK_NEAR(d.north, -20.0, 1e-9);  // +y is south

    GridFrame lust{-2.2, 1.0};  // grid north points 2.2 deg west of true north
    d = lust.fromArteryDelta(0.0, -100.0);  // 100 m grid north
    CHECK_NEAR(Geodesy::bearing(d), 360.0 - 2.2, 1e-6);
    const LocalPoint r = lust.toArteryDelta(d);
    CHECK_NEAR(r.east, 0.0, 1e-9);
    CHECK_NEAR(r.north, -100.0, 1e-9);
    CHECK_NEAR(lust.toTrueHeading(10.0), 7.8, 1e-9);
}

void testCoverage()
{
    const SensorCoverage cov = SensorCoverage::parse("100:50:0, 100:50:180");
    CHECK(cov.sectors().size() == 2);
    // CPM objects are measured from the mount of the cone they are most central in (ITU-R M.2057-1 Radar B)
    const SensorCoverage bumpers = SensorCoverage::parse("100:50:0:2.5,100:50:180:-2.5");
    CHECK_NEAR(bumpers.mountFor(0.0, LocalPoint{3.0, 60.0}).north, 2.5, 1e-9);   // front radar
    CHECK_NEAR(bumpers.mountFor(0.0, LocalPoint{3.0, -60.0}).north, -2.5, 1e-9); // rear radar

    bool threw = false;
    try {
        SensorCoverage::parse("100-120");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

void testAssociationSolvers()
{
    std::vector<ProjectedTrack> tracks(2);
    tracks[0].station = 10;
    tracks[0].state = state(at(0, 0), 10.0, 0.0, 0.0);
    tracks[1].station = 11;
    tracks[1].state = state(at(0, 2.0), 10.0, 0.0, 0.0);
    // object A sits between both tracks but closer to 11; object B only fits 10
    std::vector<ObjectClaim> objects{object(5, 1, at(0, 1.2), 10.0, 0.0, 0.0), object(5, 2, at(0, -0.8), 10.0, 0.0, 0.0),
                                     object(5, 3, at(40, 40), 10.0, 0.0, 0.0)};
    AssociationParams params;
    AssociationContext ctx{tracks, objects, params};

    const auto matches = GreedyAssociation().associate(ctx);
    CHECK(matches.size() == 2);
    std::map<size_t, StationId> byObject;
    for (const auto& m : matches) byObject[m.objectIndex] = m.station;
    CHECK(byObject[0] == 11);
    CHECK(byObject[1] == 10);
    CHECK(byObject.count(2) == 0);  // far object stays unresolved

    // an object without velocity is matched on position alone
    std::vector<ObjectClaim> noVel{object(5, 1, at(0, 0.3), 0.0, 0.0, 0.0, false)};
    AssociationContext ctx2{tracks, noVel, params};
    CHECK(GreedyAssociation().associate(ctx2).size() == 1);
}

// ── Error models (ErrorModels.h) ──────────────────────────────────────────────

void testGnssModel()
{
    GnssErrorModel m;  // VeReMi Extension
    CHECK_NEAR(m.positionSigma(), 5.0 / std::sqrt(3.0), 1e-12);
    // the error stays around the vehicle's E0: with zero jitter it converges to E0 from any start
    auto mean = [](double mu, double) { return mu; };
    double e = 0.0;
    for (int k = 0; k < 30; ++k) e = m.step(4.0, e, mean);
    CHECK_NEAR(e, 4.0, 1e-6);
    // jitter is 3 % of |E0|
    double sigmaSeen = -1.0;
    m.step(-4.0, -4.0, [&](double, double sigma) { sigmaSeen = sigma; return 0.0; });
    CHECK_NEAR(sigmaSeen, 0.12, 1e-12);
    // heading error decays with speed: exp(-0.1 * 10) at 10 m/s
    CHECK_NEAR(m.headingError(20.0, 10.0), 20.0 * std::exp(-1.0), 1e-12);
    CHECK(m.headingSigma(20.0) < m.headingSigma(0.0));
}

void testRadarModel()
{
    RadarErrorModel r;  // Jacobs et al. VTC2023-Spring
    CHECK_NEAR(r.positionVariance(0.0), 0.5 * 0.16, 1e-12);
    const double cross = 50.0 * 3.0 * ConstantsWGS84::DEG_TO_RAD;
    CHECK_NEAR(r.positionVariance(50.0), 0.5 * (0.16 + cross * cross), 1e-12);
    // no noise -> the target itself
    const LocalPoint m = r.measure(LocalPoint{0.0, 2.5}, LocalPoint{10.0, 40.0}, [](double mu, double) { return mu; });
    CHECK_NEAR(m.east, 10.0, 1e-9);
    CHECK_NEAR(m.north, 40.0, 1e-9);
    // a 3 deg azimuth error at 100 m moves the target ~5.2 m sideways
    const LocalPoint s = r.measure(LocalPoint{0.0, 0.0}, LocalPoint{0.0, 100.0},
                                   [](double mu, double sigma) { return sigma == 3.0 ? mu + 3.0 : mu; });
    CHECK_NEAR(s.east, 100.0 * std::sin(3.0 * ConstantsWGS84::DEG_TO_RAD), 1e-6);
}

// ── Scoring: Paper 1's counterless filter (Confidence.h) ─────────────────────

void testCounterlessFilter()
{
    double s = 0.5;
    const double expected[] = {0.538, 0.568, 0.591};
    for (double e : expected) {
        s = counterlessUpdate(s, 1.0, 0.0, 10.0);
        CHECK_NEAR(s, e, 5e-4);
    }
    for (int k = 0; k < 60; ++k) s = counterlessUpdate(s, 1.0, 0.0, 10.0);
    CHECK_NEAR(s, 2.0 / 3.0, 1e-6);  // steady state with support on every message
    // no evidence: back towards 0.5, from either side
    CHECK(counterlessUpdate(0.9, 0.0, 0.0, 10.0) < 0.9 && counterlessUpdate(0.9, 0.0, 0.0, 10.0) > 0.5);
    CHECK(counterlessUpdate(0.2, 0.0, 0.0, 10.0) > 0.2);
}

// ── Association ─────────────────────────────────────────────────────────────

void testGates()
{
    AssociationParams params;
    const double sg = params.sigmaGPS;
    CHECK_NEAR(SpatialAssociationUtils::computeDynamicGate(20.0, 0.0, params, 2.0, 1.0), 3.0 * std::sqrt(2 * sg * sg + 1.0), 1e-9);
    CHECK(SpatialAssociationUtils::computeDynamicGate(20.0, 0.5, params, 2.0, 1.0) >
          SpatialAssociationUtils::computeDynamicGate(20.0, 0.0, params, 2.0, 1.0));
    // the heading term shrinks with speed (VeReMi: error ~ exp(-0.1 v))
    CHECK(SpatialAssociationUtils::headingSigma(30.0, params) < SpatialAssociationUtils::headingSigma(0.0, params));

    std::map<StationId, std::deque<KinematicState>> cache;
    cache[1].push_back(state(at(0, 0), 10.0, 0.0, 0.0));
    cache[2].push_back(state(at(0, 0), 10.0, 0.0, 0.0));
    const auto tracks = TemporalTrackSynchronizer::synchronize(cache, 0.3, SyncMode::ConstantVelocity, 0.5, nullptr, {2});
    CHECK(tracks.size() == 1);
    CHECK_NEAR(tracks[0].syncDt, 0.3, 1e-12);
    CHECK_NEAR(Geodesy::toLocal(kOrigin, GeoPoint{tracks[0].state.lat, tracks[0].state.lon}).north, 3.0, 1e-6);
}

// ── Engine scenarios ────────────────────────────────────────────────────────

struct Scenario {
    TrustEngineParams params;
    std::unique_ptr<TrustEngine> engine;
    std::vector<ClaimResult> results;
    EgoView ego;

    Scenario()
    {
        params.kinematics.maxPosDev = 30.85;  // Paper 1 (CollectiveTrustService.ned)
        params.kinematics.contextSpeedWeight = 0.9874;
        params.kinematics.contextYawWeight = 0.0387;
        rebuild();
        ego.station = 1;
        ego.centre = at(0, 0);
        ego.heading = 0.0;  // north, static
    }

    void rebuild()
    {
        engine = std::make_unique<TrustEngine>(params, DeadReckoningModelFactory::create("HistoryAwareCTRA"),
                                               std::make_unique<GreedyAssociation>());
        engine->setClaimSink([this](const ClaimResult& r) { results.push_back(r); });
    }

    const ClaimResult& last() const { return results.back(); }

    /// The last result for `author`'s CAM or object `objectId`.
    const ClaimResult* lastOf(StationId author, ClaimKind kind, uint16_t objectId = 0) const
    {
        for (auto it = results.rbegin(); it != results.rend(); ++it) {
            if (it->author == author && it->kind == kind && it->objectId == objectId) return &*it;
        }
        return nullptr;
    }
};

CamClaim cam(StationId id, const GeoPoint& p, double speed, double heading, double t)
{
    CamClaim c;
    c.author = id;
    c.state = state(p, speed, heading, t);
    c.length = 5.0;
    return c;
}

CpmSnapshot cpmFrom(StationId author, const GeoPoint& centre, double heading, double t, std::vector<ObjectClaim> objects)
{
    CpmSnapshot c;
    c.author = author;
    c.time = t;
    c.heading = heading;
    c.referencePosition = Geodesy::offset(centre, heading, 2.5);  // front bumper of a 5 m car
    c.objects = std::move(objects);
    return c;
}

/// Car 7 drives north at 10 m/s, 40 m ahead of the ego at t = 0; CAMs every 0.1 s.
GeoPoint car7(double t) { return at(0.0, 40.0 + 10.0 * t); }

// Paper 1 is reproduced exactly: new sender at 1.0, then the evaluator on every message.
void testPaper1Parity()
{
    Scenario s;
    std::deque<KinematicState> history;
    BetaReputationTrustEvaluator paper1;
    auto dr = DeadReckoningModelFactory::create("HistoryAwareCTRA");
    double reference = 1.0;
    for (int k = 0; k < 12; ++k) {
        const double t = 0.3 * k;
        const KinematicState st = state(car7(t), 10.0, 0.0, t);
        s.ego.time = t;
        s.engine->onCam(cam(7, car7(t), 10.0, 0.0, t), s.ego);
        if (!history.empty()) {
            const KinematicState past = history.back();
            const auto physics = phase0_PhysicalInvariance<Wgs84CoordinatePolicy>(past, st, s.params.kinematics.speedToleranceMps);
            double err = 0, thr = 0;
            phase1_KinematicConsistency(dr.get(), history, past, st, t - past.time, s.params.kinematics, physics, err, thr);
            TrustEvaluationContext ctx{history, past, st, reference, err, physics, thr, 0};
            reference = paper1.evaluateTrust(ctx, s.params.kinematics);
        }
        history.push_back(st);
        if (history.size() > 4) history.pop_front();
        CHECK_NEAR(s.last().plausibility, reference, 1e-12);
    }
    CHECK_NEAR(s.last().plausibility, 2.0 / 3.0, 0.02);  // honest steady state of Paper 1's filter
    // a teleport (RandomPos-like) is implausible at once
    s.ego.time = 3.6;
    s.engine->onCam(cam(7, at(300.0, -200.0), 10.0, 0.0, 3.6), s.ego);
    CHECK(s.last().plausibility < 0.5);
    CHECK(s.last().state == ClaimState::Implausible);
}

// Another author's CPM that resolves to the sender confirms it.
void testPeersOnlyConfirm()
{
    Scenario s;
    // car 7 is parked 40 m east. Author 8, parked 30 m south of it, reports it. One peer (support 0.5 per
    // message) lifts C towards 0.6; it crosses 0.58 after nine confirmed messages.
    const GeoPoint p7 = at(40.0, 0.0);
    const GeoPoint p8 = at(40.0, -30.0);
    for (int k = 0; k < 12; ++k) {
        const double t = 0.3 * k;
        s.ego.time = t;
        s.engine->onCam(cam(7, p7, 0.0, 0.0, t), s.ego);
        s.engine->onCam(cam(8, p8, 0.0, 0.0, t), s.ego);
        s.engine->onCpm(cpmFrom(8, p8, 0.0, t, {object(8, 5, p7, 0.0, 0.0, t)}), s.ego);
    }
    const ClaimResult* r = s.lastOf(7, ClaimKind::Cam);
    CHECK(r && r->peers >= 1);
    CHECK(r && r->state == ClaimState::Corroborated);

    // a sender nobody else reports stays at 0.5: uncorroborated, never flagged by corroboration
    Scenario lonely;
    for (int k = 0; k < 10; ++k) {
        lonely.ego.time = 0.3 * k;
        lonely.engine->onCam(cam(9, at(-40.0, 3.0 * k), 10.0, 0.0, 0.3 * k), lonely.ego);
    }
    CHECK_NEAR(lonely.last().corroboration, 0.5, 1e-9);
    CHECK(lonely.last().state == ClaimState::Uncorroborated);
}

// Other authors only confirm: car 7 claims to be parked 40 m east of the receiver. Authors 20 and 21,
// 40 m south and north of the claimed spot and facing it, send CPMs that do not contain it; their silence is no
// evidence, so no group of vehicles can lower the confidence of another.
struct PeerRun {
    Scenario s;
    void run(bool reportIt)
    {
        const GeoPoint p7 = at(40.0, 0.0);
        for (int k = 0; k < 7; ++k) {
            const double t = 0.3 * k;
            s.ego.time = t;
            const std::vector<ObjectClaim> none;
            s.engine->onCpm(cpmFrom(20, at(40.0, -40.0), 0.0, t,
                                    reportIt ? std::vector<ObjectClaim>{object(20, 3, p7, 0.0, 0.0, t)} : none),
                            s.ego);
            s.engine->onCpm(cpmFrom(21, at(40.0, 40.0), 180.0, t, none), s.ego);
            s.engine->onCam(cam(7, p7, 0.0, 0.0, t), s.ego);
        }
    }
    const ClaimResult& last() const { return *s.lastOf(7, ClaimKind::Cam); }
};

void testSilenceIsNoEvidence()
{
    PeerRun silent;
    silent.run(false);
    CHECK(silent.last().state == ClaimState::Uncorroborated);
    CHECK(silent.last().corroboration >= 0.5);  // silence never lowers C

    PeerRun reported;  // author 20 reports the spot: it confirms (testPeersOnlyConfirm shows it corroborated)
    reported.run(true);
    CHECK(reported.last().peers >= 1);
    CHECK(reported.last().corroboration > silent.last().corroboration);
}

// Objects: a real car reported by two authors (and by its own CAMs) is corroborated, a ghost reported by one author
// is not, and an object at the receiver is the receiver itself (no LDM track).
void testObjects()
{
    Scenario s;
    for (int k = 0; k < 5; ++k) {
        const double t = 0.3 * k;
        s.ego.time = t;
        s.engine->onCam(cam(7, car7(t), 10.0, 0.0, t), s.ego);
        s.engine->onCam(cam(8, at(0.0, -30.0 + 3.0 * k), 10.0, 0.0, t), s.ego);
        s.engine->onCpm(cpmFrom(8, at(0.0, -30.0 + 3.0 * k), 0.0, t,
                                {object(8, 1, car7(t), 10.0, 0.0, t),                    // the real car ahead
                                 object(8, 2, at(-8.0, 60.0 + 3.0 * k), 10.0, 0.0, t),   // a ghost
                                 object(8, 3, at(0.0, 0.0), 0.0, 0.0, t)}),             // the receiver itself
                        s.ego);
        s.engine->onCpm(cpmFrom(9, at(5.0, 10.0), 0.0, t, {object(9, 4, car7(t), 10.0, 0.0, t)}), s.ego);
    }
    CHECK(s.lastOf(8, ClaimKind::Object, 1)->state == ClaimState::Corroborated);
    CHECK(s.lastOf(8, ClaimKind::Object, 2)->state == ClaimState::Uncorroborated);
    CHECK(s.lastOf(8, ClaimKind::Object, 2)->peers == 0);
    CHECK(s.lastOf(8, ClaimKind::Object, 3)->ldmTrack == 0);
}

// A report that cannot be physical is implausible at once (Paper 1's single-frame ceiling).
void testObjectAdmissibility()
{
    Scenario s;
    s.ego.time = 0.0;
    s.engine->onCpm(cpmFrom(8, at(0.0, -30.0), 0.0, 0.0, {object(8, 1, at(0.0, 150.0), 10.0, 0.0, 0.0),  // 180 m away
                                                          object(8, 2, at(0.0, 20.0), 90.0, 0.0, 0.0)}), // 90 m/s
                    s.ego);
    CHECK(s.lastOf(8, ClaimKind::Object, 1)->state == ClaimState::Implausible);
    CHECK(s.lastOf(8, ClaimKind::Object, 2)->state == ClaimState::Implausible);
}

// The CPM reference pose must agree with the author's own CAM track; if not, the objects it places are penalised.
void testCpmPoseAgainstCam()
{
    Scenario honest;
    Scenario liar;
    for (int k = 0; k < 3; ++k) {
        const double t = 0.3 * k;
        const GeoPoint pos = at(0.0, -30.0 + 3.0 * k);
        for (Scenario* s : {&honest, &liar}) {
            s->ego.time = t;
            s->engine->onCam(cam(8, pos, 10.0, 0.0, t), s->ego);
        }
        const std::vector<ObjectClaim> objects{object(8, 1, at(-60.0, 10.0 + 3.0 * k), 10.0, 0.0, t)};
        honest.engine->onCpm(cpmFrom(8, pos, 0.0, t, objects), honest.ego);
        liar.engine->onCpm(cpmFrom(8, at(20.0, -30.0 + 3.0 * k), 0.0, t, objects), liar.ego);  // 20 m off
    }
    CHECK(honest.lastOf(8, ClaimKind::Object, 1)->plausibility >= 0.5);
    CHECK(liar.lastOf(8, ClaimKind::Object, 1)->state == ClaimState::Implausible);

    // A CAM trajectory already judged implausible (a falsified speed) is not a reference: the CPM's real objects
    // are not penalised for the CAM's lie.
    Scenario speedLiar;
    for (int k = 0; k < 4; ++k) {
        const double t = 0.3 * k;
        const GeoPoint pos = at(0.0, -30.0 + 3.0 * k);  // truly 10 m/s
        speedLiar.ego.time = t;
        speedLiar.engine->onCam(cam(8, pos, 45.0, 0.0, t), speedLiar.ego);  // claims 45 m/s
        speedLiar.engine->onCpm(cpmFrom(8, pos, 0.0, t + 0.2, {object(8, 1, at(-60.0, 10.0 + 3.0 * k), 10.0, 0.0, t + 0.2)}),
                                speedLiar.ego);
    }
    CHECK(speedLiar.lastOf(8, ClaimKind::Cam)->state == ClaimState::Implausible);
    CHECK(speedLiar.lastOf(8, ClaimKind::Object, 1)->plausibility >= 0.5);
}

void testLdm()
{
    WorldMap ldm(4.0);
    // a CAM body, then another author's CPM object on it (resolved)
    const uint64_t body = ldm.addCam(7, state(at(0, 50), 10.0, 0.0, 0.0));
    CHECK(ldm.addObject(object(8, 3, at(0.5, 50.2), 10.0, 0.0, 0.0), 7, 1, 4.7) == body);
    CHECK(ldm.independentSources(body, {7, 1}, -1.0, 1.0) == 1);  // author 8 only: not the sender, not the receiver
    // two authors report the same unresolved object -> one track, two sources
    const uint64_t a = ldm.addObject(object(8, 5, at(30, 100), 0.0, 0.0, 0.0), -1, 1, 4.7);
    const uint64_t b = ldm.addObject(object(9, 11, at(31, 101), 0.0, 0.0, 0.0), -1, 1, 4.7);
    CHECK(a != 0 && a == b);
    CHECK(ldm.independentSources(a, {8}, -1.0, 1.0) == 1);
    // the same author renames the object (id churn) -> still the same track
    CHECK(ldm.addObject(object(8, 6, at(30.2, 100.1), 0.0, 0.0, 0.6), -1, 1, 4.7) == a);
    // a ghost far away -> its own track, nobody else on it
    const uint64_t ghost = ldm.addObject(object(8, 99, at(-60, 20), 0.0, 0.0, 0.0), -1, 1, 4.7);
    CHECK(ghost != a && ghost != body);
    CHECK(ldm.independentSources(ghost, {8, 1}, -1.0, 1.0) == 0);
    CHECK(ldm.independentSources(ghost, {1}, -1.0, 1.0) == 1);  // its author only
    // the receiver itself is not a track
    CHECK(ldm.addObject(object(8, 1, at(0, 0), 0.0, 0.0, 0.0), 1, 1, 4.7) == 0);
    ldm.evict(10.0);
    CHECK(ldm.size() == 0);
}

}  // namespace

int main()
{
    const std::vector<std::pair<const char*, std::function<void()>>> tests{
        {"geodesy", testGeodesy},
        {"grid frame", testGridFrame},
        {"sensor mounts (CPM generation)", testCoverage},
        {"GNSS error model (VeReMi Extension)", testGnssModel},
        {"radar error model (Jacobs et al.)", testRadarModel},
        {"counterless filter", testCounterlessFilter},
        {"gates", testGates},
        {"gated assignment", testAssociationSolvers},
        {"LDM association and provenance", testLdm},
        {"Paper 1 parity", testPaper1Parity},
        {"peers confirm", testPeersOnlyConfirm},
        {"silent authors are no evidence", testSilenceIsNoEvidence},
        {"objects", testObjects},
        {"object admissibility", testObjectAdmissibility},
        {"CPM pose against CAM", testCpmPoseAgainstCam},
    };
    for (const auto& t : tests) {
        const int before = g_failures;
        t.second();
        std::printf("%s %s\n", g_failures == before ? "[ok]  " : "[FAIL]", t.first);
    }
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
