#ifndef I_CONFIDENCE_EVALUATOR_H_
#define I_CONFIDENCE_EVALUATOR_H_

#include "MotionTypes.h"
#include "MechanismConstants.h"
#include "EngineConfig.h"
#include "Checks.h"
#include <string>
#include <deque>


struct ConfidenceContext {
    const std::deque<KinematicState>& history;
    const KinematicState& pastState;
    const KinematicState& currentState;
    double lastConfidence;
    double distError;
    PhysicalInvarianceResult physicsResult = {};

    double dynamicMaxPosDev    = 0.0;   // Context-aware dynamic threshold [m]
    long   senderId            = 0;     // Sender identifier for stateful scorers
};

class IConfidenceEvaluator
{
public:
    virtual ~IConfidenceEvaluator() = default;
    virtual double evaluateConfidence(const ConfidenceContext& context,
                                 const EngineConfig& params) = 0;
    virtual std::string getName() const = 0;
    virtual void reset() {}
};

#endif // I_CONFIDENCE_EVALUATOR_H_
