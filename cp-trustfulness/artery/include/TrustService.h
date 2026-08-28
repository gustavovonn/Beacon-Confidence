#ifndef TRUST_SERVICE_H_
#define TRUST_SERVICE_H_

#include "../../include/IDeadReckoningModel.h"
#include "../../include/ITrustEvaluator.h"
#include "../../include/DeadReckoningModels.h"
#include "../../include/TrustEvaluators.h"
#include <map>
#include <deque>
#include <memory>
#include <artery/application/ItsG5BaseService.h>
#include <artery/application/CaObject.h>
#include <omnetpp/coutvector.h>

namespace artery
{

class TrustService : public ItsG5BaseService
{
public:
    virtual ~TrustService();
    void initialize() override;
    void trigger() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signal, omnetpp::cObject* obj, omnetpp::cObject* details) override;

    void populateDataBase(long stationId);

    void colorEntityInSUMO(long stationId);

    void updateEntitiesColorsInSUMO(long stationId, double trustScore);

    void logContextInCSV(double now, long stationId, double speed, double yawRate, double dt, double distError,
                         double centroidDeviation);

    bool requiresListener() const override;
    void finish() override;

private:
    omnetpp::simsignal_t m_sigCamReceived;

    std::map<long, std::deque<KinematicState>> m_cache;
    std::map<long, double> m_scores;

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
