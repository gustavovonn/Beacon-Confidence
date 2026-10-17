#ifndef SCORER_H_
#define SCORER_H_

#include "EngineConfig.h"
#include "IConfidenceEvaluator.h"
#include "ConfidenceEvaluators.h"
#include "Checks.h"
#include <map>
#include <memory>
#include <string>

/**
 * @brief Context-aware reputation scorer (paper Sec. IV-B).
 *
 * Holds one scalar confidence score S in [0,1] per tracked sender and applies the
 * configured evidence filter. The multi-tier verification itself (consistency
 * check, dynamic threshold, predictive check) lives in Checks.h; this
 * class owns the per-sender posterior and dispatches evidence to the evaluator.
 */
class Scorer {
public:
    Scorer(const std::string& evaluatorName, const EngineConfig& config)
        : m_cfg(config)
    {
        m_evaluator = ConfidenceEvaluatorFactory::create(evaluatorName, m_cfg.historyWindow);
    }

    /// Current confidence for a sender.
    double confidenceOf(long senderId) const
    {
        auto it = m_confidence.find(senderId);
        return (it != m_confidence.end()) ? it->second : 1.0;
    }

    void setConfidence(long senderId, double value) { m_confidence[senderId] = value; }

    /// Dispatch evidence to the evaluator and store the updated posterior.
    double evaluate(const ConfidenceContext& context)
    {
        const double updated = m_evaluator->evaluateConfidence(context, m_cfg);
        m_confidence[context.senderId] = updated;
        return updated;
    }

    const std::map<long, double>& confidences() const { return m_confidence; }

    IConfidenceEvaluator* evaluator() const { return m_evaluator.get(); }

    void reset()
    {
        m_confidence.clear();
        if (m_evaluator) {
            m_evaluator->reset();
        }
    }

private:
    std::map<long, double> m_confidence;
    std::unique_ptr<IConfidenceEvaluator> m_evaluator;
    EngineConfig m_cfg;
};

#endif // SCORER_H_
