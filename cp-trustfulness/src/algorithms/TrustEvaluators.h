#ifndef TRUST_EVALUATORS_H_
#define TRUST_EVALUATORS_H_

#include "ITrustEvaluator.h"
#include <memory>
#include <deque>

namespace artery
{

class ExponentialTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(std::deque<KinematicState> history, double currentTrust, double distError, const MotionVector& state,
            double actualSpeed, double actualHeading,
            const TrustEvaluationParams& params) override;
    std::string getName() const override { return "Exponential"; }
};

class MultiVectorTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(std::deque<KinematicState> history, double currentTrust, double distError, const MotionVector& state,
                         double actualSpeed, double actualHeading,
                         const TrustEvaluationParams& params) override;
    std::string getName() const override { return "MultiVector"; }
};

class MahalanobisTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(std::deque<KinematicState> history, double currentTrust, double distError, const MotionVector& state,
                         double actualSpeed, double actualHeading,
                         const TrustEvaluationParams& params) override;
    std::string getName() const override { return "Mahalanobis"; }
};

class BetaReputationTrustEvaluator : public ITrustEvaluator
{
public:
    double evaluateTrust(std::deque<KinematicState> history, double currentTrust, double distError, const MotionVector& state,
                         double actualSpeed, double actualHeading,
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
