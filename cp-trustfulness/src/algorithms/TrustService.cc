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
    // TODO: remover essa lógica de trigger e não apenas descartar o estado previsto
    omnetpp::SimTime now = omnetpp::simTime();

    for (auto& pair : m_predictions) {
        MotionVector& motionVector = pair.second;
        KinematicState nextState;

        long stationId = pair.first;
        auto it = m_cache.find(stationId);
        if (it == m_cache.end()) {
            std::deque<KinematicState> hist;
            m_cache[stationId] = hist;
        }
        std::deque<KinematicState> history = m_cache[stationId];

        double dt = (now - motionVector.time).dbl();
        if (m_drModel && dt > 0.0) {
            m_drModel->predict(history, motionVector, nextState, dt);
        }
    }
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

    // Extract position in degrees (latitude/longitude are in 1e-7 degrees)
    double lat = cam->cam.camParameters.basicContainer.referencePosition.latitude * 1e-7;
    double lon = cam->cam.camParameters.basicContainer.referencePosition.longitude * 1e-7;

    // Extract high frequency container
    const auto& hf = cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

    // Extract speed in m/s (speed is in 0.01 m/s)
    double speed = hf.speed.speedValue * 0.01;

    // Extract heading in degrees (heading is in 0.1 degrees)
    double heading = hf.heading.headingValue * 0.1;

    // Extract acceleration in m/s^2 (longitudinalAcceleration is in 0.1 m/s^2)
    double accel = hf.longitudinalAcceleration.longitudinalAccelerationValue * 0.1;

    // Extract yaw rate in rad/s (yawRate is in 0.01 deg/s)
    double yawRate = hf.yawRate.yawRateValue * 0.01 * M_PI / 180.0;

    // Logs that only outputs in Qtenv mode
    // EV_INFO << "⬇ [CAM IN] Node " << stationId << " claims:"
    //         << " Pos(" << lat << ", " << lon << "),"
    //         << " Speed: " << speed << "m/s,"
    //         << " Heading: " << heading << "deg" << std::endl;

    omnetpp::SimTime now = omnetpp::simTime();

    auto it = m_predictions.find(stationId);
    if (it == m_predictions.end()) {
        // New Vehicle incoming
        MotionVector state;
        state.stationId = stationId;
        state.lat = lat;
        state.lon = lon;
        state.speed = speed;
        state.heading = heading;
        state.accel = accel;
        state.yawRate = yawRate;
        state.time = now;

        // TODO: rever essa parte
        state.trustScore = 1.0;
        state.predictionError = 0.0;
        state.centroidDeviation = 0.0;

        KinematicState snapshot;
        snapshot.lat = lat;
        snapshot.lon = lon;
        snapshot.speed = speed;
        snapshot.heading = heading;
        snapshot.time = now;

        std::deque<KinematicState> history;
        history.push_back(snapshot);
        m_cache[stationId] = history;

        m_predictions[stationId] = state;
        this->populateDataBase(stationId);
        this->colorEntityInSUMO(stationId);
        return;
    }
    MotionVector& state = it->second;

    // 1. Compute DR state prediction error
    double distError = m_drModel->computeError(state, lat, lon, speed, heading);
    state.predictionError = distError;

    std::deque<KinematicState> history = m_cache[stationId];

    // --- Phase 2: Centroid Deviation Check ---
    double centroidDeviation = 0.0;
    if (!history.empty()) {
        double sumLat = 0.0;
        double sumLon = 0.0;
        for (const auto& snap : history) {
            sumLat += snap.lat;
            sumLon += snap.lon;
        }
        double cLat = sumLat / history.size();
        double cLon = sumLon / history.size();

        double latRad1 = cLat * M_PI / 180.0;
        double latRad2 = lat * M_PI / 180.0;
        double lonRad1 = cLon * M_PI / 180.0;
        double lonRad2 = lon * M_PI / 180.0;
        double x = (lonRad2 - lonRad1) * std::cos((latRad1 + latRad2) / 2.0);
        double y = (latRad2 - latRad1);
        centroidDeviation = 6371000.0 * std::sqrt(x * x + y * y);
    }
    state.centroidDeviation = centroidDeviation;

    // Compute time gap since last received CAM for this vehicle
    double dt = (now - state.time).dbl();

    // 2. Evaluate raw trust score using pluggable strategy
    double oldTrust = state.trustScore;
    state.trustScore = m_trustEvaluator->evaluateTrust(history, state.trustScore, distError, state, speed, heading, m_params);

    this->logContextInCSV(now.dbl(), stationId, speed, yawRate, dt, distError, centroidDeviation);

    // ====================================================================
    // Updates SQLite/OMNet++ vectors.
    // Only the trust is currently being used to calculate final simulation stats.
    // ====================================================================
    if (m_trustVectors.find(stationId) != m_trustVectors.end()) {
        m_trustVectors[stationId]->record(state.trustScore);
    }
    if (m_dtVectors.find(stationId) != m_dtVectors.end()) {
        m_dtVectors[stationId]->record(dt);
    }
    if (m_distErrorVectors.find(stationId) != m_distErrorVectors.end()) {
        m_distErrorVectors[stationId]->record(distError);
    }

    this->updateEntitiesColorsInSUMO(stationId, state);

    // Update state history with actual received CAM values
    state.lat = lat;
    state.lon = lon;
    state.speed = speed;
    state.heading = heading;
    state.accel = accel;
    state.yawRate = yawRate;
    state.time = now;

    KinematicState snapshot;
    snapshot.lat = lat;
    snapshot.lon = lon;
    snapshot.speed = speed;
    snapshot.heading = heading;
    snapshot.time = now;
    history.push_back(snapshot);
    while (history.size() > static_cast<size_t>(m_historySize)) {
        history.pop_front();
    }
}

void TrustService::populateDataBase(long stationId) {
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

void TrustService::colorEntityInSUMO(long stationId) {
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

void TrustService::updateEntitiesColorsInSUMO(long stationId, const MotionVector& state) {
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
                        if (state.trustScore < 0.5) {
                            traciApi->vehicle.setColor(senderTraciId, libsumo::TraCIColor(255, 0, 0, 255)); // TURN RED
                        } // else if (oldTrust < 0.5 && state.trustScore >= 0.7) {
                        //     traciApi->vehicle.setColor(senderTraciId, libsumo::TraCIColor(0, 255, 0, 255)); // RECOVERED GREEN
                        // }
                    } catch (...) {}
                }
            }
        }
    }
}

void TrustService::logContextInCSV(double now, long stationId, double speed,
    double yawRate, double dt, double distError, double centroidDeviation) {
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
