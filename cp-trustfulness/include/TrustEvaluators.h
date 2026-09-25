#ifndef TRUST_EVALUATORS_H_
#define TRUST_EVALUATORS_H_

#include "ITrustEvaluator.h"
#include <memory>
#include <deque>
#include <map>

namespace artery
{

class BetaReputationTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(const TrustEvaluationContext& context,
                         const TrustEvaluationParams& params) override;

    std::string getName() const override { return "BetaReputation"; }
};

class EwmaTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(const TrustEvaluationContext& context,
                         const TrustEvaluationParams& params) override;

    std::string getName() const override { return "EWMA"; }
};

class MajorityVoteTrustEvaluator : public ITrustEvaluator
{
private:
    std::map<long, std::deque<bool>> m_verdictWindows;
    size_t m_windowSize;

public:
    explicit MajorityVoteTrustEvaluator(size_t windowSize = 10)
        : m_windowSize(windowSize) {}

    double evaluateTrust(const TrustEvaluationContext& context,
                         const TrustEvaluationParams& params) override;

    std::string getName() const override { return "MajorityVote"; }
    void reset() override { m_verdictWindows.clear(); }
};

class TrustEvaluatorFactory
{
public:
    static std::unique_ptr<ITrustEvaluator> create(const std::string& name,
                                                   size_t windowSize = 10);
};

} // namespace artery

#endif // TRUST_EVALUATORS_H_
