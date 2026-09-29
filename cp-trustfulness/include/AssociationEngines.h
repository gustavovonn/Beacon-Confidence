#ifndef ASSOCIATION_ENGINES_H_
#define ASSOCIATION_ENGINES_H_

#include "IAssociationEngine.h"
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

/// Gated assignment: gated pairs accepted by ascending cost, each CAM track and object at most once.
class GreedyAssociation : public IAssociationEngine {
public:
    std::vector<AssociationMatch> associate(const AssociationContext& ctx) override;
    std::string getName() const override { return "Greedy"; }
};

}  // namespace artery

#endif  // ASSOCIATION_ENGINES_H_
