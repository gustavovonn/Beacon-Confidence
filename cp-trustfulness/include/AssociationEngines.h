#ifndef ASSOCIATION_ENGINES_H_
#define ASSOCIATION_ENGINES_H_

#include "IAssociationEngine.h"
#include <memory>
#include <string>
#include <vector>

namespace artery {

namespace SpatialAssociationUtils {
/**
 * Gate radius for comparing two position estimates of the same physical object:
 *
 *   kappa * sqrt(gpsSources * sigmaGPS^2 + sensorVariance + (v * dt * sigmaHeading(v))^2) + aMax * dt^2 / 2
 *
 * gpsSources counts the independent GNSS fixes behind the two estimates, sensorVariance sums the per-axis
 * variances of the radar measurements (RadarErrorModel::positionVariance). CAM track vs CPM object: the CAM
 * sender's GNSS, the CPM author's GNSS and the author's radar.
 */
double computeDynamicGate(double speed, double dt, const AssociationParams& params, double gpsSources,
                          double sensorVariance);
/// Heading error [rad] at speed v (VeReMi Extension: decays exponentially with speed).
double headingSigma(double speed, const AssociationParams& params);
double calculateMatchConfidence(double cost);
}  // namespace SpatialAssociationUtils

/// Gated global assignment by the Hungarian algorithm (Kuhn-Munkres): the assignment of objects to CAM
/// tracks that minimises the total cost, with each track and each object used at most once. Pairs outside
/// the gate are forbidden, so an object with no gated track stays unresolved.
class HungarianAssociation : public IAssociationEngine {
public:
    std::vector<AssociationMatch> associate(const AssociationContext& ctx) override;
    std::string getName() const override { return "Hungarian"; }
};

/// Gated greedy nearest neighbour: repeatedly takes the cheapest remaining pair inside the gate, with
/// each track and each object used at most once. Kept as the ablation baseline for the assignment, since
/// greedy and Hungarian differ exactly where two objects compete for one track; it is NOT the production
/// engine and must never be described as Hungarian.
class GreedyAssociation : public IAssociationEngine {
public:
    std::vector<AssociationMatch> associate(const AssociationContext& ctx) override;
    std::string getName() const override { return "Greedy"; }
};

/// Builds the engine named by `name` ("hungarian" or "greedy"); defaults to Hungarian.
std::unique_ptr<IAssociationEngine> makeAssociationEngine(const std::string& name);

}  // namespace artery

#endif  // ASSOCIATION_ENGINES_H_
