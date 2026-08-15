#ifndef TRUST_SERVICE_H_
#define TRUST_SERVICE_H_

#include <artery/application/ItsG5BaseService.h>
#include <artery/application/CaObject.h>
#include "IDeadReckoningModel.h"
#include "ITrustEvaluator.h"
#include "DeadReckoningModels.h"
#include "TrustEvaluators.h"
#include <omnetpp/coutvector.h>
#include <map>
#include <memory>

namespace artery
{

class TrustService : public ItsG5BaseService
{
public:
    virtual ~TrustService();
    void initialize() override;
    void trigger() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signal, omnetpp::cObject* obj, omnetpp::cObject* details) override;
    bool requiresListener() const override;
    void finish() override;

private:
    omnetpp::simsignal_t m_sigCamReceived;
    std::map<long, VehicleState> m_neighbors;

    // Selected modular strategy implementations
    std::unique_ptr<IDeadReckoningModel> m_drModel;
    std::unique_ptr<ITrustEvaluator> m_trustEvaluator;

    // Evaluation Threshold Parameters
    TrustEvaluationParams m_params;
    int m_historySize = 5;

    // OMNeT++ Output Vectors for logging trust metrics
    std::map<long, omnetpp::cOutVector*> m_trustVectors;
    std::map<long, omnetpp::cOutVector*> m_dtVectors;
    std::map<long, omnetpp::cOutVector*> m_distErrorVectors;
};

} // namespace artery

#endif // TRUST_SERVICE_H_
