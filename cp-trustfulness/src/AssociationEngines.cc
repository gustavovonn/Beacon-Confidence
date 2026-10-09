#include "../include/AssociationEngines.h"
#include "../include/CoordinatePolicies.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>

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
double positionVariance(const KinematicState& track, double dt, const ObjectClaim& object,
                               const AssociationParams& params)
{
    const double headingTerm = track.speed * dt * headingSigma(track.speed, params);
    return object.sensorVariance + 2.0 * params.sigmaGPS * params.sigmaGPS + headingTerm * headingTerm;
}

double pairCost(const KinematicState& track, double dt, const ObjectClaim& object,
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

constexpr double kForbidden = 1e9;  ///< cost of a pair outside the gate; never part of a valid assignment

/// Cost matrix of every track/object pair. Pairs outside the gate get `kForbidden`. `residual` holds the
/// distance normalised by that pair's gate, which is what the plausibility check reads afterwards.
struct CostMatrix {
    size_t rows = 0;  // tracks
    size_t cols = 0;  // objects
    std::vector<double> cost;
    std::vector<double> residual;

    double at(size_t r, size_t c) const { return cost[r * cols + c]; }
    double res(size_t r, size_t c) const { return residual[r * cols + c]; }
};

CostMatrix buildCostMatrix(const AssociationContext& ctx)
{
    CostMatrix m;
    m.rows = ctx.tracks.size();
    m.cols = ctx.objects.size();
    m.cost.assign(m.rows * m.cols, kForbidden);
    m.residual.assign(m.rows * m.cols, 1.0);
    for (size_t i = 0; i < m.rows; ++i) {
        const ProjectedTrack& track = ctx.tracks[i];
        for (size_t j = 0; j < m.cols; ++j) {
            const ObjectClaim& object = ctx.objects[j];
            // the CAM sender's GNSS, the author's GNSS and the author's radar measurement
            const double gate = SpatialAssociationUtils::computeDynamicGate(track.state.speed, track.syncDt,
                                                                            ctx.params, 2.0, object.sensorVariance);
            const double dist = Wgs84CoordinatePolicy::distance(track.state.lat, track.state.lon,
                                                                object.position.lat, object.position.lon);
            m.residual[i * m.cols + j] = gate > 0.0 ? dist / gate : 1.0;
            if (dist <= gate) {
                m.cost[i * m.cols + j] = SpatialAssociationUtils::pairCost(track.state, track.syncDt, object,
                                                                           ctx.params);
            }
        }
    }
    return m;
}

/**
 * Hungarian algorithm (Kuhn-Munkres) with potentials, O(n^3). Rows are tracks, columns are objects; the
 * matrix is padded to a square internally. Returns, for each column, the row assigned to it, or SIZE_MAX.
 */
std::vector<size_t> solveAssignment(const CostMatrix& m)
{
    const size_t n = std::max(m.rows, m.cols);
    if (n == 0) {
        return {};
    }
    const double kPad = kForbidden;
    auto cell = [&](size_t r, size_t c) {
        return (r < m.rows && c < m.cols) ? m.at(r, c) : kPad;
    };

    // u, v: potentials; way: alternating-path predecessors; assignment[c] = row matched to column c
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<size_t> assignment(n + 1, n), way(n + 1, n);

    for (size_t i = 0; i < n; ++i) {
        assignment[n] = i;
        size_t j0 = n;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<char> used(n + 1, false);
        do {
            used[j0] = true;
            const size_t i0 = assignment[j0];
            double delta = std::numeric_limits<double>::infinity();
            size_t j1 = n;
            for (size_t j = 0; j < n; ++j) {
                if (used[j]) {
                    continue;
                }
                const double cur = cell(i0, j) - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (size_t j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[assignment[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (assignment[j0] != n);
        do {
            const size_t j1 = way[j0];
            assignment[j0] = assignment[j1];
            j0 = j1;
        } while (j0 != n);
    }

    std::vector<size_t> columnToRow(m.cols, SIZE_MAX);
    for (size_t j = 0; j < m.cols; ++j) {
        const size_t r = assignment[j];
        if (r < m.rows) {
            columnToRow[j] = r;
        }
    }
    return columnToRow;
}

}  // namespace

std::vector<AssociationMatch> HungarianAssociation::associate(const AssociationContext& ctx)
{
    std::vector<AssociationMatch> matches;
    if (ctx.tracks.empty() || ctx.objects.empty()) {
        return matches;
    }
    const CostMatrix m = buildCostMatrix(ctx);
    const std::vector<size_t> columnToRow = solveAssignment(m);
    for (size_t j = 0; j < m.cols; ++j) {
        const size_t i = columnToRow[j];
        // a padded or forbidden cell is not a real pair: the object stays unresolved
        if (i == SIZE_MAX || m.at(i, j) >= kForbidden) {
            continue;
        }
        AssociationMatch match;
        match.station = ctx.tracks[i].station;
        match.objectIndex = j;
        match.normalisedResidual = m.res(i, j);
        matches.push_back(match);
    }
    return matches;
}

/**
 * Greedy gated nearest neighbour. Sorts every gated pair by cost and takes them in order, skipping any
 * whose track or object is already used. This is the ablation baseline: it differs from the Hungarian
 * solution precisely when a locally cheapest pair forces a worse global assignment, which is the case
 * the crossed-assignment test exercises.
 */
std::vector<AssociationMatch> GreedyAssociation::associate(const AssociationContext& ctx)
{
    std::vector<AssociationMatch> matches;
    if (ctx.tracks.empty() || ctx.objects.empty()) {
        return matches;
    }
    const CostMatrix m = buildCostMatrix(ctx);

    struct Pair {
        double cost;
        size_t row;
        size_t col;
    };
    std::vector<Pair> pairs;
    for (size_t i = 0; i < m.rows; ++i) {
        for (size_t j = 0; j < m.cols; ++j) {
            if (m.at(i, j) < kForbidden) {
                pairs.push_back(Pair{m.at(i, j), i, j});
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        // ties broken by index so the result does not depend on the sort's stability
        return a.cost != b.cost ? a.cost < b.cost : std::tie(a.row, a.col) < std::tie(b.row, b.col);
    });

    std::vector<char> usedRow(m.rows, false), usedCol(m.cols, false);
    for (const Pair& p : pairs) {
        if (usedRow[p.row] || usedCol[p.col]) {
            continue;
        }
        usedRow[p.row] = usedCol[p.col] = true;
        AssociationMatch match;
        match.station = ctx.tracks[p.row].station;
        match.objectIndex = p.col;
        match.normalisedResidual = m.res(p.row, p.col);
        matches.push_back(match);
    }
    std::sort(matches.begin(), matches.end(),
              [](const AssociationMatch& a, const AssociationMatch& b) { return a.objectIndex < b.objectIndex; });
    return matches;
}

std::unique_ptr<IAssociationEngine> makeAssociationEngine(const std::string& name)
{
    if (name == "greedy" || name == "Greedy") {
        return std::make_unique<GreedyAssociation>();
    }
    return std::make_unique<HungarianAssociation>();
}

}  // namespace artery
