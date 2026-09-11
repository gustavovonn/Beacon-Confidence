#ifndef TRUST_EVALUATORS_H_
#define TRUST_EVALUATORS_H_

#include "ITrustEvaluator.h"
#include <memory>
#include <deque>

namespace artery
{

struct PhysicalInvarianceResult {
    bool failed = false;
    double excessRatio = 0.0;
    double safeSpeed = 0.0;
    double dt = 0.1;
};

class BetaReputationTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(const TrustEvaluationContext& context,
                         const TrustEvaluationParams& params) override;

    std::string getName() const override { return "BetaReputation"; }

private:
    PhysicalInvarianceResult checkPhysicalInvariants(const TrustEvaluationContext& context,
                                                    const TrustEvaluationParams& params) const;
};

class TrustEvaluatorFactory
{
public:
    static std::unique_ptr<ITrustEvaluator> create(const std::string& name);
};

} // namespace artery

#endif // TRUST_EVALUATORS_H_
