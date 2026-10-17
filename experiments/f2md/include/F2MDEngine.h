#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <filesystem>

#include "BinaryMessageFormat.h"

// F2MD Dependencies
#include "F2MDParameters.h"
#include "mdChecks/LegacyChecks.h"
#include "mdChecks/CaTChChecks.h"
#include "mdChecks/ExperiChecks.h"
#include "mdApplications/ThresholdApp.h"
#include "mdApplications/BehavioralApp.h"
#include "mdApplications/AggregationApp.h"
#include "mdApplications/ExperiApp.h"
#include "mdBase/NodeTable.h"
#include "mdSupport/networkLinksLib/LinkControl.h"
#include "BasicSafetyMessage.h"

// -----------------------------------------------------------------------------
// Data Structures
inline double global_sim_time = 0.0;



// -----------------------------------------------------------------------------
// Binary Message Extractor (Adapted for BSM mapping)
// -----------------------------------------------------------------------------
class F2MDParquetExtractor {
public:
    static std::vector<BasicSafetyMessage> extractMessages(const std::string& path) {
        if (BinaryMessageExtractor::isBinaryFile(path) || path.ends_with(".bin") || path.ends_with(".dat")) {
            return BinaryMessageExtractor::extractF2mdMessages(path);
        }
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                if (entry.path().extension() == ".bin" || entry.path().extension() == ".dat") {
                    return BinaryMessageExtractor::extractF2mdMessages(path);
                }
            }
        }
        std::cerr << "[!] Error: Unsupported dataset format. Please provide a .bin trace "
                     "(convert with scripts/export_parquet_to_bin.py).\n";
        return {};
    }
};

// -----------------------------------------------------------------------------
class F2MDEvaluationEngine {
private:
    F2MDParameters params;
    NodeTable detectedNodes;
    std::unordered_map<LAddress::L2Type, veins::Coord> realDynamicMap;
    LinkControl* linkControl;
    std::unique_ptr<MDApplication> app;
    bool silent;
    std::string checkerName;
    std::string appName;

    // "Observer" node static context
    veins::Coord myPosition = {0, 0, 0};
    veins::Coord mySpeed    = {0, 0, 0};
    veins::Coord myHeading  = {1, 0, 0};
    veins::Coord mySize     = {2.0, 4.5, 0};
    LAddress::L2Type myId   = 0;

public:
    F2MDEvaluationEngine(LinkControl* lc, const std::string& checker = "f2md-legacy", const std::string& application = "threshold", bool silentMode = false, const std::string& dataset_dir = "") 
        : linkControl(lc), silent(silentMode), checkerName(checker), appName(application) {
        
        if (appName == "behavioral" || appName == "behavioral-app") {
            app = std::make_unique<BehavioralApp>(1, 0.5);
        } else if (appName == "aggregation" || appName == "aggregation-app") {
            app = std::make_unique<AggrigationApp>(1, 0.5, 0.5, 10.0, 5);
        } else if (appName == "experi" || appName == "experi-app") {
            app = std::make_unique<ExperiApp>(1, 10.0, 5, 20.0);
        } else if (appName == "threshold" || appName == "threshold-app") {
            app = std::make_unique<ThresholdApp>(1, 0.28125);
        } else if (appName == "ml" || appName == "ml-app" || appName == "mlp") {
            std::cerr << "[!] Error: MachineLearningApp is not implemented yet (pending manual validation).\n";
            throw std::runtime_error("MachineLearningApp is not implemented yet (pending manual validation).");
        } else {
            // Default to ThresholdApp
            app = std::make_unique<ThresholdApp>(1, 0.28125);
        }

        if (!silent) {
            std::cout << "rcvTime,sender,flagged\n";
        }
    }

    BsmCheck runStage1(BasicSafetyMessage& bsm, double* stage1_us = nullptr) {
        myId = bsm.receiver_id;
        myPosition = bsm.recv_pos;
        mySpeed = bsm.recv_speed;
        myHeading = bsm.recv_heading;
        realDynamicMap[bsm.getSenderPseudonym()] = bsm.getSenderPos();
        veins::Coord myLimits = {params.MIN_MAX_SPEED, params.MIN_MAX_ACCEL, params.MIN_MAX_DECEL};

        if (checkerName == "f2md-catch" || checkerName == "catch") {
            veins::Coord myPositionConfidence = {0,0,0};
            veins::Coord myHeadingConfidence = {0,0,0};
            CaTChChecks checker(1, bsm.receiver_pseudo, myPosition, myPositionConfidence, myHeading, myHeadingConfidence, mySize, myLimits, linkControl, &realDynamicMap, myId, &params);
            
            auto t0 = std::chrono::high_resolution_clock::now();
            BsmCheck chk = checker.CheckBSM(&bsm, &detectedNodes);
            auto t1 = std::chrono::high_resolution_clock::now();
            if (stage1_us) *stage1_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            return chk;
        } else if (checkerName == "f2md-experi" || checkerName == "experi") {
            veins::Coord myPositionConfidence = {0,0,0};
            veins::Coord myHeadingConfidence = {0,0,0};
            ExperiChecks checker(1, bsm.receiver_pseudo, myPosition, myPositionConfidence, myHeading, myHeadingConfidence, mySize, myLimits, linkControl, &realDynamicMap, myId, &params);
            
            auto t0 = std::chrono::high_resolution_clock::now();
            BsmCheck chk = checker.CheckBSM(&bsm, &detectedNodes);
            auto t1 = std::chrono::high_resolution_clock::now();
            if (stage1_us) *stage1_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            return chk;
        } else {
            // Default to LegacyChecks
            LegacyChecks checker(1, bsm.receiver_pseudo, myPosition, mySpeed, myHeading, mySize, myLimits, linkControl, &realDynamicMap, myId, &params);
            
            auto t0 = std::chrono::high_resolution_clock::now();
            BsmCheck chk = checker.CheckBSM(&bsm, &detectedNodes);
            auto t1 = std::chrono::high_resolution_clock::now();
            if (stage1_us) *stage1_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            return chk;
        }
    }

    bool runStage2(BasicSafetyMessage& bsm, BsmCheck& bsmCheck, double* stage2_us = nullptr) {
        if (appName == "aggregation" || appName == "aggregation-app" || appName == "experi" || appName == "experi-app") {
            if (!detectedNodes.includes(bsm.getSenderPseudonym())) {
                if (stage2_us) *stage2_us = 0.0;
                return false;
            }
        }
        
        auto t0 = std::chrono::high_resolution_clock::now();
        bool flagged = app->CheckNodeForReport(bsm.receiver_pseudo, &bsm, &bsmCheck, &detectedNodes);
        auto t1 = std::chrono::high_resolution_clock::now();
        if (stage2_us) *stage2_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        return flagged;
    }

    void updateHistory(BasicSafetyMessage& bsm, BsmCheck& bsmCheck) {
        if (!detectedNodes.includes(bsm.getSenderPseudonym())) {
            NodeHistory newNode(bsm.getSenderPseudonym());
            newNode.addBSM(bsm);
            MDMHistory newMDM(bsm.getSenderPseudonym());
            newMDM.addBsmCheck(bsmCheck, 1);
            newMDM.initKalman(&bsm, 1);
            detectedNodes.put(bsm.getSenderPseudonym(), newNode, newMDM, nullptr, nullptr);
        } else {
            detectedNodes.getNodeHistoryAddr(bsm.getSenderPseudonym())->addBSM(bsm);
            detectedNodes.getMDMHistoryAddr(bsm.getSenderPseudonym())->addBsmCheck(bsmCheck, 1);
        }
    }

    bool processMessage(BasicSafetyMessage& bsm, double* stage1_us = nullptr, double* stage2_us = nullptr) {
        global_sim_time = bsm.getArrivalTime().dbl();

        BsmCheck bsmCheck = runStage1(bsm, stage1_us);
        bool flagged = runStage2(bsm, bsmCheck, stage2_us);

        updateHistory(bsm, bsmCheck);

        return flagged;
    }
};
