#pragma once
#include "../include/IDeadReckoningModel.h"
#include "../include/DeadReckoningModels.h"
#include "../include/CoordinatePolicies.h"
#include "../include/PhysicalInvariance.h"
#include "../include/TrustEvaluators.h"
#include "../include/CheckerPipeline.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <cmath>
#include <memory>
#include <algorithm>
#include <chrono>

// -----------------------------------------------------------------------------
// Utilities & Data Structures
// -----------------------------------------------------------------------------
#include "BinaryMessageFormat.h"

#include <filesystem>
#include <string_view>

struct PrevState {
    double time;
    double speed;
    double heading;
};

// -----------------------------------------------------------------------------
// Binary Message Extractor
// -----------------------------------------------------------------------------
class ParquetMessageExtractor {
public:
    static std::vector<RawMessage> extractMessages(const std::string& path) {
        if (BinaryMessageExtractor::isBinaryFile(path) || path.ends_with(".bin") || path.ends_with(".dat")) {
            auto msgs = BinaryMessageExtractor::extractRawMessages(path);
            computeDerivedKinematics(msgs);
            return msgs;
        }
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                if (entry.path().extension() == ".bin" || entry.path().extension() == ".dat") {
                    auto msgs = BinaryMessageExtractor::extractRawMessages(path);
                    computeDerivedKinematics(msgs);
                    return msgs;
                }
            }
        }
        std::cerr << "[!] Error: Unsupported dataset format. Please provide a .bin trace "
                     "(convert with scripts/export_parquet_to_bin.py).\n";
        return {};
    }

private:
    static void computeDerivedKinematics(std::vector<RawMessage>& all_messages) {
        std::map<long, PrevState> prev_states;
        
        for (auto& msg : all_messages) {
            auto p_it = prev_states.find(msg.sender);
            if (p_it != prev_states.end()) {
                double dt = msg.rcvTime - p_it->second.time;
                if (dt <= 0.0) dt = 0.1;
                
                msg.accel = (msg.speed - p_it->second.speed) / dt;
                
                double diff = msg.heading - p_it->second.heading;
                diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
                msg.yawRate = (diff * M_PI / 180.0) / dt;
            }
            prev_states[msg.sender] = {msg.rcvTime, msg.speed, msg.heading};
        }
    }
};

// -----------------------------------------------------------------------------
// Trust Evaluation Engine
// -----------------------------------------------------------------------------
class TrustEvaluationEngine {
private:
    std::unique_ptr<IDeadReckoningModel> drModel;
    std::unique_ptr<ITrustEvaluator> trustEvaluator;
    TrustEvaluationParams params;
    bool silent;
    size_t historySize = ConstantsApp::DEFAULT_HISTORY_WINDOW;
    
    std::map<long, std::deque<KinematicState>> m_cache;
    std::map<long, double> m_scores;

public:
    TrustEvaluationEngine(const std::string& drModelName, const std::string& trustModelName, 
                          double maxPosDev = 4.3506, double contextSpeedWeight = 5.2264, double contextYawWeight = 0.6536,
                          bool silentMode = false, size_t histSize = ConstantsApp::DEFAULT_HISTORY_WINDOW) {
        silent = silentMode;
        historySize = histSize;
        drModel = DeadReckoningModelFactory::create(drModelName, "cartesian");
        
        trustEvaluator = TrustEvaluatorFactory::create(trustModelName);

        // values from offline training or overridden by CLI
        params.maxPosDev = maxPosDev;
        params.contextSpeedWeight = contextSpeedWeight;
        params.contextYawWeight = contextYawWeight;

        if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << "rcvTime,sender,score,distError\n";
#endif
        }
    }
    
    bool runStage1(const RawMessage& msg, double* stage1_us = nullptr) {
        KinematicState currentState;
        currentState.time = msg.rcvTime;
        currentState.lon = msg.pos_x;
        currentState.lat = msg.pos_y;
        currentState.speed = msg.speed;
        currentState.heading = msg.heading;
        currentState.accel = msg.accel;
        currentState.yawRate = msg.yawRate;

        auto it = m_cache.find(msg.sender);
        if (it == m_cache.end()) {
            m_scores[msg.sender] = 1.0;
            std::deque<KinematicState> newHistory;
            newHistory.push_back(currentState);
            m_cache[msg.sender] = newHistory;
            if (stage1_us) *stage1_us = 0.0;
            return false;
        }

        auto& history = it->second;
        KinematicState pastState = history.back();
        
        double dt = currentState.time - pastState.time;
        if (dt <= 0.0) dt = 0.1;

        auto t0 = std::chrono::high_resolution_clock::now();
        PhysicalInvarianceResult physicsResult = phase0_PhysicalInvariance<CartesianCoordinatePolicy>(
            pastState, currentState, params.speedToleranceMps);
        double distError = 0.0, dynamicMaxPosDev = 0.0;
        phase1_KinematicConsistency(
            drModel.get(), history, pastState, currentState, dt, params, physicsResult,
            distError, dynamicMaxPosDev);
        auto t1 = std::chrono::high_resolution_clock::now();

        history.push_back(currentState);
        while (history.size() > historySize) history.pop_front();

        if (stage1_us) *stage1_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        return true;
    }

    bool runStage2(const RawMessage& msg, double distError = 0.0, double* stage2_us = nullptr) {
        auto it = m_cache.find(msg.sender);
        std::deque<KinematicState> dummyHistory;
        KinematicState dummyState;
        dummyState.time = msg.rcvTime;
        dummyState.speed = msg.speed;
        dummyState.yawRate = msg.yawRate;

        const auto& history = (it != m_cache.end()) ? it->second : dummyHistory;
        KinematicState pastState = (!history.empty()) ? history.back() : dummyState;

        PhysicalInvarianceResult physicsResult = phase0_PhysicalInvariance<CartesianCoordinatePolicy>(
            pastState, dummyState, params.speedToleranceMps);
        double dynamicMaxPosDev = computeDynamicThreshold(physicsResult, pastState.yawRate, params);

        TrustEvaluationContext context {
            history,
            pastState,
            dummyState,
            (m_scores.find(msg.sender) != m_scores.end()) ? m_scores[msg.sender] : 1.0,
            distError,
            physicsResult,
            dynamicMaxPosDev,
            msg.sender
        };

        auto t0 = std::chrono::high_resolution_clock::now();
        double newScore = trustEvaluator->evaluateTrust(context, params);
        auto t1 = std::chrono::high_resolution_clock::now();
        m_scores[msg.sender] = newScore;

        if (stage2_us) *stage2_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        return (newScore < 0.5);
    }

    bool processMessage(const RawMessage& msg, double* stage1_us = nullptr, double* stage2_us = nullptr) {
        KinematicState currentState;
        currentState.time = msg.rcvTime;
        currentState.lon = msg.pos_x;
        currentState.lat = msg.pos_y;
        currentState.speed = msg.speed;
        currentState.heading = msg.heading;
        currentState.accel = msg.accel;
        currentState.yawRate = msg.yawRate;

        auto it = m_cache.find(msg.sender);
        if (it == m_cache.end()) {
            m_scores[msg.sender] = 1.0;
            std::deque<KinematicState> newHistory;
            newHistory.push_back(currentState);
            m_cache[msg.sender] = newHistory;
            if (stage1_us) *stage1_us = 0.0;
            if (stage2_us) *stage2_us = 0.0;
            if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.sender << ",1.0,0.0\n";
#endif
            }
            return false;
        }

        auto& history = it->second;
        KinematicState pastState = history.back();
        
        double dt = currentState.time - pastState.time;
        if (dt <= 0.0) dt = 0.1;

        // Stage 1: Phase 0, Phase 1 Checkers Pipeline
        auto t0 = std::chrono::high_resolution_clock::now();
        PhysicalInvarianceResult physicsResult = phase0_PhysicalInvariance<CartesianCoordinatePolicy>(
            pastState, currentState, params.speedToleranceMps);
        double distError = 0.0, dynamicMaxPosDev = 0.0;
        phase1_KinematicConsistency(
            drModel.get(), history, pastState, currentState, dt, params, physicsResult,
            distError, dynamicMaxPosDev);
        auto t1 = std::chrono::high_resolution_clock::now();

        // Stage 2: Scorer Evidence Dispatch & Update
        TrustEvaluationContext context {
            history,
            pastState,
            currentState,
            m_scores[msg.sender],
            distError,
            physicsResult,
            dynamicMaxPosDev,
            msg.sender
        };

        auto t2 = std::chrono::high_resolution_clock::now();
        double newScore = trustEvaluator->evaluateTrust(context, params);
        auto t3 = std::chrono::high_resolution_clock::now();
        m_scores[msg.sender] = newScore;

        if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.sender << "," << newScore << "," 
                      << distError << "\n";
#endif
        }

        history.push_back(currentState);
        while (history.size() > historySize) {
            history.pop_front();
        }

        if (stage1_us) *stage1_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (stage2_us) *stage2_us = std::chrono::duration<double, std::micro>(t3 - t2).count();

        return (newScore < 0.5);
    }
};
