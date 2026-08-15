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
    m_params.maxSpeedDev = par("maxSpeedDeviation").doubleValue();
    m_params.maxHeadingDev = par("maxHeadingDeviation").doubleValue();
    m_params.penaltyFactor = par("trustPenaltyFactor").doubleValue();
    m_params.recoveryRate = par("trustRecoveryRate").doubleValue();
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
    omnetpp::SimTime now = omnetpp::simTime();

    for (auto& pair : m_neighbors) {
        VehicleState& state = pair.second;
        double dt = (now - state.lastCamTime).dbl();
        if (m_drModel && dt > 0.0) {
            m_drModel->predict(state, dt);
        }
    }
}

void TrustService::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject* details)
{
    // to react only when receiving CAM messages
    if (signal == m_sigCamReceived && obj != nullptr) {
        auto caObj = dynamic_cast<CaObject*>(obj);
        if (caObj) {
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

            auto it = m_neighbors.find(stationId);
            if (it == m_neighbors.end()) {
                // New Vehicle incoming
                VehicleState state;
                state.stationId = stationId;
                state.lastLat = lat;
                state.lastLon = lon;
                state.lastSpeed = speed;
                state.lastHeading = heading;
                state.lastAccel = accel;
                state.lastYawRate = yawRate;
                state.lastCamTime = now;
                state.expectedLat = lat;
                state.expectedLon = lon;
                state.expectedSpeed = speed;
                state.expectedHeading = heading;
                state.trustScore = 1.0;
                state.lastPredictionError = 0.0;
                state.lastCentroidDeviation = 0.0;

                CamSnapshot snapshot;
                snapshot.lat = lat;
                snapshot.lon = lon;
                snapshot.speed = speed;
                snapshot.heading = heading;
                snapshot.time = now;
                state.history.push_back(snapshot);

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

                m_neighbors[stationId] = state;

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
            } else {
                VehicleState& state = it->second;

                // 1. Compute DR state prediction error
                double distError = m_drModel->computeError(state, lat, lon, speed, heading);
                state.lastPredictionError = distError;

                // --- Phase 2: Centroid Deviation Check ---
                double centroidDeviation = 0.0;
                if (!state.history.empty()) {
                    double sumLat = 0.0;
                    double sumLon = 0.0;
                    for (const auto& snap : state.history) {
                        sumLat += snap.lat;
                        sumLon += snap.lon;
                    }
                    double cLat = sumLat / state.history.size();
                    double cLon = sumLon / state.history.size();
                    
                    double latRad1 = cLat * M_PI / 180.0;
                    double latRad2 = lat * M_PI / 180.0;
                    double lonRad1 = cLon * M_PI / 180.0;
                    double lonRad2 = lon * M_PI / 180.0;
                    double x = (lonRad2 - lonRad1) * std::cos((latRad1 + latRad2) / 2.0);
                    double y = (latRad2 - latRad1);
                    centroidDeviation = 6371000.0 * std::sqrt(x * x + y * y);
                }
                state.lastCentroidDeviation = centroidDeviation;

                // Compute time gap since last received CAM for this vehicle
                double dt = (now - state.lastCamTime).dbl();

                // 2. Evaluate raw trust score using pluggable strategy
                double oldTrust = state.trustScore;
                state.trustScore = m_trustEvaluator->evaluateTrust(state.trustScore, distError, state, speed, heading, m_params);

                // ======================================================================
                // CSV Log Context Dataset for Offline Optimization
                // ======================================================================
                {
                    std::lock_guard<std::mutex> lock(g_csvMutex);
                    if (!g_csvFile.is_open()) {
                        system("mkdir -p results/offline-training");
                        g_csvFile.open("results/offline-training/trust_context_dataset.csv", std::ios::out | std::ios::trunc);
                    }
                    if (!g_csvHeaderWritten) {
                        g_csvFile << "time,nodeId,speed,yawRate,dt,distError,centroidDeviation\n";
                        g_csvHeaderWritten = true;
                    }
                    g_csvFile << now.dbl() << "," << stationId << "," << speed << "," 
                              << yawRate << "," << dt << "," << distError << "," 
                              << centroidDeviation << "\n";
                }

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

                // Update state history with actual received CAM values
                state.lastLat = lat;
                state.lastLon = lon;
                state.lastSpeed = speed;
                state.lastHeading = heading;
                state.lastAccel = accel;
                state.lastYawRate = yawRate;
                state.lastCamTime = now;
                
                CamSnapshot snapshot;
                snapshot.lat = lat;
                snapshot.lon = lon;
                snapshot.speed = speed;
                snapshot.heading = heading;
                snapshot.time = now;
                state.history.push_back(snapshot);
                while (state.history.size() > static_cast<size_t>(m_historySize)) {
                    state.history.pop_front();
                }
            }
        }
    }
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
