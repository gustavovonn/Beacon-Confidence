#ifndef TRUST_EVALUATORS_H_
#define TRUST_EVALUATORS_H_

#include "ITrustEvaluator.h"
#include <memory>
#include <deque>

namespace artery
{

class BetaReputationTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(const TrustEvaluationContext& context,
                         const TrustEvaluationParams& params) override;

    std::string getName() const override { return "BetaReputation"; }
};

class TrustEvaluatorFactory
{
public:
    static std::unique_ptr<ITrustEvaluator> create(const std::string& name);
};

} // namespace artery

#endif // TRUST_EVALUATORS_H_
