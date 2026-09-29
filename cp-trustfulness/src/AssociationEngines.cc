#include "../include/AssociationEngines.h"
#include "../include/CoordinatePolicies.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery {

namespace SpatialAssociationUtils {

double headingSigma(double speed, const AssociationParams& params)
{
    return params.sigmaHeading * std::exp(-params.headingSpeedDecay * speed);
}

double computeDynamicGate(double speed, double dt, const AssociationParams& params, double gpsSources,
                          double sensorVariance)
{
    const double headingTerm = speed * dt * headingSigma(speed, params);
    const double sigmaPos = std::sqrt(gpsSources * params.sigmaGPS * params.sigmaGPS + sensorVariance +
                                      headingTerm * headingTerm);
    return params.kappaGate * sigmaPos + 0.5 * params.aMax * dt * dt;
}

/// Variance per axis of the CAM-track vs CPM-object difference: two GNSS fixes, the author's radar measurement
/// and the heading error over the projection horizon.
static double positionVariance(const KinematicState& track, double dt, const ObjectClaim& object,
                               const AssociationParams& params)
{
    const double headingTerm = track.speed * dt * headingSigma(track.speed, params);
    return object.sensorVariance + 2.0 * params.sigmaGPS * params.sigmaGPS + headingTerm * headingTerm;
}

static double pairCost(const KinematicState& track, double dt, const ObjectClaim& object,
                       const AssociationParams& params)
{
    const double dist = Wgs84CoordinatePolicy::distance(track.lat, track.lon, object.position.lat, object.position.lon);
    double cost = (dist * dist) / positionVariance(track, dt, object, params);
    if (object.hasVelocity) {
        const double dv = track.speed - object.speed;
        cost += params.lambdaV * (dv * dv) / (params.sigmaVel * params.sigmaVel);
    }
    return cost;
}

double calculateMatchConfidence(double cost)
{
    return std::exp(-0.5 * cost);
}

}  // namespace SpatialAssociationUtils

namespace {

struct Candidate {
    double cost;
    size_t track;
    size_t object;
};

/// All track/object pairs inside the dynamic gate and above the minimum confidence.
std::vector<Candidate> gatedCandidates(const AssociationContext& ctx)
{
    std::vector<Candidate> candidates;
    for (size_t i = 0; i < ctx.tracks.size(); ++i) {
        const ProjectedTrack& track = ctx.tracks[i];
        for (size_t j = 0; j < ctx.objects.size(); ++j) {
            const ObjectClaim& object = ctx.objects[j];
            // CAM sender's GNSS + author's GNSS + author's radar
            const double gate = SpatialAssociationUtils::computeDynamicGate(track.state.speed, track.syncDt, ctx.params,
                                                                            2.0, object.sensorVariance);
            const double dist = Wgs84CoordinatePolicy::distance(track.state.lat, track.state.lon,
                                                                object.position.lat, object.position.lon);
            if (dist > gate) {
                continue;
            }
            const double cost = SpatialAssociationUtils::pairCost(track.state, track.syncDt, object, ctx.params);
            // the confidence cut follows from the gate (cost <= kappa^2), so it is never tighter than the gate
            const double minConfidence = std::exp(-0.5 * ctx.params.kappaGate * ctx.params.kappaGate);
            if (SpatialAssociationUtils::calculateMatchConfidence(cost) >= minConfidence) {
                candidates.push_back({cost, i, j});
            }
        }
    }
    return candidates;
}

AssociationMatch makeMatch(const AssociationContext& ctx, size_t trackIndex, size_t objectIndex)
{
    AssociationMatch match;
    match.station = ctx.tracks[trackIndex].station;
    match.objectIndex = objectIndex;
    return match;
}

}  // namespace

std::vector<AssociationMatch> GreedyAssociation::associate(const AssociationContext& ctx)
{
    std::vector<Candidate> candidates = gatedCandidates(ctx);
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });

    std::vector<bool> trackUsed(ctx.tracks.size(), false);
    std::vector<bool> objectUsed(ctx.objects.size(), false);
    std::vector<AssociationMatch> matches;
    for (const Candidate& c : candidates) {
        if (trackUsed[c.track] || objectUsed[c.object]) {
            continue;
        }
        trackUsed[c.track] = true;
        objectUsed[c.object] = true;
        matches.push_back(makeMatch(ctx, c.track, c.object));
    }
    return matches;
}

}  // namespace artery
