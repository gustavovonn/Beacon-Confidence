#include "TrustService.h"
#include "artery/application/Middleware.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/IdentityRegistry.h"
#include <omnetpp/cmodule.h>
#include <omnetpp/cdisplaystring.h>
#include <omnetpp/csimulation.h>
#include <iostream>
#include <algorithm>

#include <algorithm>
#include <fstream>
#include <mutex>
#include <cstdlib>

using namespace omnetpp;

namespace {
    std::mutex g_csvMutex;
    std::ofstream g_csvFile;
    bool g_csvHeaderWritten = false;
}

namespace artery
{

static std::string findTraciIdByStationId(uint32_t stationId)
{
    cModule* idRegMod = omnetpp::getSimulation()->getModuleByPath("World.idRegistry");
    if (idRegMod) {
        auto* idReg = dynamic_cast<IdentityRegistry*>(idRegMod);
        if (idReg) {
            auto identity = idReg->lookup<IdentityRegistry::application>(stationId);
            if (identity) {
                return identity->traci;
            }
        }
    }
    int maxId = omnetpp::getSimulation()->getLastComponentId();
    for (int id = 1; id <= maxId; ++id) {
        cModule* mod = omnetpp::getSimulation()->getModule(id);
        if (mod) {
            auto middleware = dynamic_cast<Middleware*>(mod);
            if (middleware && middleware->getIdentity().application == stationId) {
                return middleware->getIdentity().traci;
            }
        }
    }
    return "";
}

Define_Module(TrustService)

TrustService::~TrustService()
{
    for (auto& pair : m_trustVectors) {
        if (pair.second) {
            drop(pair.second);
            delete pair.second;
        }
    }
    m_trustVectors.clear();
}

void TrustService::initialize()
{
    ItsG5BaseService::initialize();

    // Algorithms parameters
    std::string drAlg = par("drAlgorithm").stringValue();
    std::string trustAlg = par("trustAlgorithm").stringValue();

    m_drModel = DeadReckoningModelFactory::create(drAlg);
    m_trustEvaluator = TrustEvaluatorFactory::create(trustAlg);

    m_params.maxPosDev = par("maxPositionDeviation").doubleValue();
    m_params.contextSpeedWeight = par("contextSpeedWeight").doubleValue();
    m_params.contextYawWeight = par("contextYawWeight").doubleValue();
    m_historySize = par("historySize").intValue();

    // Qtenv log
    // EV_INFO << "TrustService initialized with DR Model [" << m_drModel->getName()
    //         << "] and Trust Evaluator [" << m_trustEvaluator->getName() << "]" << std::endl;

    // CAM registering    
    m_sigCamReceived = cComponent::registerSignal("CamReceived");
    subscribe(m_sigCamReceived);
 
    // TraCI controller (it's for the visualization coloring)
    auto controller = getFacilities().get_mutable_ptr<traci::VehicleController>();
    if (controller) {
        std::string vId = controller->getVehicleId();
        if (!vId.empty() && std::all_of(vId.begin(), vId.end(), ::isdigit)) {
            recordScalar("sumoVehicleID", std::atol(vId.c_str()));
        }
    }
}

void TrustService::trigger()
{
    // Predict-and-discard loop has been removed. Predictions are made on-demand in receiveSignal.
}

KinematicState* etsi_cam_unwrap(const auto& asn1_cam)
{
    KinematicState* kinematicState = new KinematicState();

    // Extract position in degrees (latitude/longitude are in 1e-7 degrees)
    kinematicState->lat = asn1_cam->cam.camParameters.basicContainer.referencePosition.latitude * 1e-7;
    kinematicState->lon = asn1_cam->cam.camParameters.basicContainer.referencePosition.longitude * 1e-7;

    // Extract high frequency container
    const auto& hf = asn1_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

    // Extract speed in m/s (speed is in 0.01 m/s)
    kinematicState->speed = hf.speed.speedValue * 0.01;

    // Extract heading in degrees (heading is in 0.1 degrees)
    kinematicState->heading = hf.heading.headingValue * 0.1;

    // Extract acceleration in m/s^2 (longitudinalAcceleration is in 0.1 m/s^2)
    kinematicState->accel = hf.longitudinalAcceleration.longitudinalAccelerationValue * 0.1;

    // Extract yaw rate in rad/s (yawRate is in 0.01 deg/s)
    kinematicState->yawRate = hf.yawRate.yawRateValue * 0.01 * M_PI / 180.0;

    kinematicState->time = omnetpp::simTime().dbl();

    return kinematicState;
}

double calculateCentroidDeviation(const std::deque<KinematicState>& state_history,
                                  double receivedLat, double receivedLon)
{
    if (state_history.empty()) return 0.0;

    double sumLat = 0.0;
    double sumLon = 0.0;

    for (const auto& snap : state_history) {
        sumLat += snap.lat;
        sumLon += snap.lon;
    }

    double cLat = sumLat / state_history.size();
    double cLon = sumLon / state_history.size();

    double latRad1 = cLat * M_PI / 180.0;
    double latRad2 = receivedLat * M_PI / 180.0;
    double lonRad1 = cLon * M_PI / 180.0;
    double lonRad2 = receivedLon * M_PI / 180.0;
    double x = (lonRad2 - lonRad1) * std::cos((latRad1 + latRad2) / 2.0);
    double y = (latRad2 - latRad1);

    return (6371000.0 * std::sqrt(x * x + y * y));
}

void TrustService::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject* details)
{
    // to react only when receiving CAM messages
    if (signal != m_sigCamReceived || obj == nullptr) {
        return;
    }
    auto caObj = dynamic_cast<CaObject*>(obj);
    if (!caObj) {
        return;
    }
    const auto& cam = caObj->asn1();
    long stationId = cam->header.stationID;

    KinematicState* receivedKinematicState = etsi_cam_unwrap(cam);

    auto it = m_cache.find(stationId);

    // New Vehicle incoming
    if (it == m_cache.end()) {

        // TODO: review first score attribution
        m_scores[stationId] = 1.0;

        // update the just received kinematic state inside the window history
        std::deque<KinematicState> history;
        history.push_back(*receivedKinematicState);
        m_cache[stationId] = history;

        this->populateDataBase(stationId);
        this->colorEntityInSUMO(stationId);
        delete receivedKinematicState;
        return;
    }

    std::deque<KinematicState>& history = m_cache[stationId];
    const KinematicState& pastKinematicState = history.back();

    // 1. Compute time gap since last received CAM for this vehicle
    double dt = receivedKinematicState->time - pastKinematicState.time;

    // 2. Predict Spatial State
    PredictedState expectedState;
    if (m_drModel && dt > 0.0) {
        m_drModel->predict(history, pastKinematicState, expectedState, dt);
    } else {
        expectedState.lat = pastKinematicState.lat;
        expectedState.lon = pastKinematicState.lon;
        expectedState.speed = pastKinematicState.speed;
        expectedState.heading = pastKinematicState.heading;
    }

    // 3. Compute Distance Error
    double distError = 0.0;
    if (m_drModel) {
        distError = m_drModel->computeError(expectedState, *receivedKinematicState);
    }

    // 4. Centroid Deviation
    double centroidDeviation = calculateCentroidDeviation(history, 
        receivedKinematicState->lat, receivedKinematicState->lon);

    // 5. Evaluate Trust
    double lastScore = m_scores[stationId];
    TrustEvaluationContext ctx {
        history, 
        pastKinematicState, 
        *receivedKinematicState, 
        lastScore, 
        distError, 
        centroidDeviation
    };
    
    if (m_trustEvaluator) {
        m_scores[stationId] = m_trustEvaluator->evaluateTrust(ctx, m_params);
    }

    this->logContextInCSV(omnetpp::simTime().dbl(), stationId, receivedKinematicState->speed, receivedKinematicState->yawRate, dt, distError, centroidDeviation);

    // ====================================================================
    // Updates SQLite/OMNet++ vectors.
    // Only the trust is currently being used to calculate final simulation stats.
    // ====================================================================
    if (m_trustVectors.find(stationId) != m_trustVectors.end()) {
        m_trustVectors[stationId]->record(m_scores[stationId]);
    }
    if (m_dtVectors.find(stationId) != m_dtVectors.end()) {
        m_dtVectors[stationId]->record(dt);
    }
    if (m_distErrorVectors.find(stationId) != m_distErrorVectors.end()) {
        m_distErrorVectors[stationId]->record(distError);
    }

    this->updateEntitiesColorsInSUMO(stationId, m_scores[stationId]);

    // Update state history
    history.push_back(*receivedKinematicState);

    while (history.size() > static_cast<size_t>(m_historySize)) {
        history.pop_front();
    }
    
    delete receivedKinematicState;
}

void TrustService::populateDataBase(long stationId)
{
    // ====================================================================
    // Creates SQLite/OMNet++ vectors and push the first measurement.
    // Only the trust is currently being used to calculate final simulation stats.
    // ====================================================================
    std::string vecName = "TrustScore_Node_" + std::to_string(stationId);
    cOutVector* vec = new cOutVector(vecName.c_str());
    take(vec);
    vec->record(1.0);
    m_trustVectors[stationId] = vec;

    std::string dtVecName = "CamDeltaT_Node_" + std::to_string(stationId);
    cOutVector* dtVec = new cOutVector(dtVecName.c_str());
    take(dtVec);
    dtVec->record(0.1);
    m_dtVectors[stationId] = dtVec;

    std::string errVecName = "DistError_Node_" + std::to_string(stationId);
    cOutVector* errVec = new cOutVector(errVecName.c_str());
    take(errVec);
    errVec->record(0.0);
    m_distErrorVectors[stationId] = errVec;
}

void TrustService::colorEntityInSUMO(long stationId)
{
    // ======================================================================
    // SUMO COLORING: vehicle starts GREEN
    // ======================================================================
    auto controller = getFacilities().get_mutable_ptr<traci::VehicleController>();
    if (controller) {
        auto traciApi = controller->getTraCI();
        if (traciApi) {
            std::string senderTraciId = findTraciIdByStationId(stationId);
            if (!senderTraciId.empty()) {
                auto activeVehicles = traciApi->vehicle.getIDList();
                if (std::find(activeVehicles.begin(), activeVehicles.end(), senderTraciId) != activeVehicles.end()) {
                    try {
                        traciApi->vehicle.setColor(senderTraciId, libsumo::TraCIColor(0, 255, 0, 255));
                    } catch (...) {}
                }
            }
        }
    }
}

void TrustService::updateEntitiesColorsInSUMO(long stationId, double trustScore)
{
    // ======================================================================
    // SIMULATION COLORING: updating to RED if trust is below 0.5
    // ======================================================================
    auto controller = getFacilities().get_mutable_ptr<traci::VehicleController>();
    if (controller) {
        auto traciApi = controller->getTraCI();
        if (traciApi) {
            std::string senderTraciId = findTraciIdByStationId(stationId);
            if (!senderTraciId.empty()) {
                auto activeVehicles = traciApi->vehicle.getIDList();
                if (std::find(activeVehicles.begin(), activeVehicles.end(), senderTraciId) != activeVehicles.end()) {
                    try {
                        if (trustScore < 0.5) {
                            traciApi->vehicle.setColor(senderTraciId, libsumo::TraCIColor(255, 0, 0, 255)); // TURN RED
                        } // else if (oldTrust < 0.5 && trustScore >= 0.7) {
                        //     traciApi->vehicle.setColor(senderTraciId, libsumo::TraCIColor(0, 255, 0, 255)); // RECOVERED GREEN
                        // }
                    } catch (...) {}
                }
            }
        }
    }
}

void TrustService::logContextInCSV(double now, long stationId, double speed,
    double yawRate, double dt, double distError, double centroidDeviation)
{
    // ======================================================================
    // CSV Log Context Dataset for Offline Optimization
    // ======================================================================
    std::lock_guard<std::mutex> lock(g_csvMutex);
    if (!g_csvFile.is_open()) {
        system("mkdir -p results/offline-training");
        g_csvFile.open("results/offline-training/trust_context_dataset.csv", std::ios::out | std::ios::trunc);
    }
    if (!g_csvHeaderWritten) {
        g_csvFile << "time,nodeId,speed,yawRate,dt,distError,centroidDeviation\n";
        g_csvHeaderWritten = true;
    }
    g_csvFile << now << "," << stationId << "," << speed << ","
            << yawRate << "," << dt << "," << distError << ","
            << centroidDeviation << "\n";
}

bool TrustService::requiresListener() const
{
    // reduced scope to have only the monitor working
    return false;
}

void TrustService::finish()
{
    ItsG5BaseService::finish();
    for (auto& pair : m_trustVectors) {
        if (pair.second) {
            drop(pair.second);
            delete pair.second;
        }
    }
    m_trustVectors.clear();

    for (auto& pair : m_dtVectors) {
        if (pair.second) {
            drop(pair.second);
            delete pair.second;
        }
    }
    m_dtVectors.clear();

    for (auto& pair : m_distErrorVectors) {
        if (pair.second) {
            drop(pair.second);
            delete pair.second;
        }
    }
    m_distErrorVectors.clear();
}

} // namespace artery
