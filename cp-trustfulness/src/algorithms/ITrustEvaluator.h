#ifndef I_TRUST_EVALUATOR_H_
#define I_TRUST_EVALUATOR_H_

#include "IDeadReckoningModel.h"
#include <string>

namespace artery
{

struct TrustEvaluationParams {
    double maxPosDev = 5.0;       // meters (acts as baseline intercept)
    
    // Context-Aware Dynamic Threshold parameters
    double contextSpeedWeight = 1.20;
    double contextYawWeight = 0.035;
};

class ITrustEvaluator
{
public:
    virtual ~ITrustEvaluator() = default;

    // Evaluate and return updated trust score in range [0.0, 1.0]
    virtual double evaluateTrust(double currentTrust, double distError, const VehicleState& state,
                                 double actualSpeed, double actualHeading,
                                 const TrustEvaluationParams& params) = 0;

    virtual std::string getName() const = 0;
};

} // namespace artery

#endif // I_TRUST_EVALUATOR_H_
