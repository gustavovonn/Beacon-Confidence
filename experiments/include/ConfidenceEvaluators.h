#ifndef CONFIDENCE_EVALUATORS_H_
#define CONFIDENCE_EVALUATORS_H_

#include "IConfidenceEvaluator.h"
#include <memory>
#include <deque>
#include <map>

class BetaReputationEvaluator : public IConfidenceEvaluator
{
public:
    double evaluateConfidence(const ConfidenceContext& context,
                         const EngineConfig& params) override;

    std::string getName() const override { return "BetaReputation"; }
};

class EwmaEvaluator : public IConfidenceEvaluator
{
public:
    double evaluateConfidence(const ConfidenceContext& context,
                         const EngineConfig& params) override;

    std::string getName() const override { return "EWMA"; }
};

class MajorityVoteEvaluator : public IConfidenceEvaluator
{
private:
    std::map<long, std::deque<bool>> m_verdictWindows;
    size_t m_windowSize;

public:
    explicit MajorityVoteEvaluator(size_t windowSize = 10)
        : m_windowSize(windowSize) {}

    double evaluateConfidence(const ConfidenceContext& context,
                         const EngineConfig& params) override;

    std::string getName() const override { return "MajorityVote"; }
    void reset() override { m_verdictWindows.clear(); }
};

class ConfidenceEvaluatorFactory
{
public:
    static std::unique_ptr<IConfidenceEvaluator> create(const std::string& name,
                                                   size_t windowSize = 10);
};

#endif // CONFIDENCE_EVALUATORS_H_
