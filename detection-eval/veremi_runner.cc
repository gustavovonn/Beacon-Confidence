#include "MotionModels.h"
#include "CoordinatePolicies.h"
#include "Checks.h"
#include "BeaconConfidenceEngine.h"
#include "ReceivedVector.h"
#include "MotionVectorAdapter.h"
#include "ParquetTraceReader.h"
#include "ConfidenceEvaluators.h"
#include "Constants.h"
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
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <filesystem>
#include <string_view>


// Compile-time switch for the per-message CSV stream.
inline constexpr bool kEmitMessageOutput = true;

// -----------------------------------------------------------------------------
// Trace loading
//
// ParquetTraceReader decodes straight into the engine's motion-vector model.
// -----------------------------------------------------------------------------
static std::vector<ReceivedVector> loadMotionVectors(const std::string& fpath)
{
    return ParquetTraceReader::read(fpath);
}


// -----------------------------------------------------------------------------
// Runner front-end over the shared BeaconConfidenceEngine
// -----------------------------------------------------------------------------
class RunnerEngine {
private:
    BeaconConfidenceEngine<CartesianCoordinatePolicy> m_engine;
    std::map<long, double> m_minScores;
    bool silent;

public:
    RunnerEngine(const std::string& motionModelName, const std::string& evaluatorName,
                 const EngineConfig& engineConfig, bool silentMode = false)
        : m_engine(motionModelName, evaluatorName, engineConfig), silent(silentMode) {}

    void reset() {
        m_engine.reset();
        m_minScores.clear();
    }

    const std::map<long, double>& getMinScores() const {
        return m_minScores;
    }

    void processMessage(const ReceivedVector& rv) {
        const ConfidenceResult res = m_engine.onMotionVector(rv.sender, rv.mv);

        switch (res.disposition) {
        case VectorDisposition::Duplicate:
            return;

        case VectorDisposition::FirstSighting:
            if (m_minScores.find(rv.sender) == m_minScores.end()) {
                m_minScores[rv.sender] = 1.0;
            }
            if (!silent && kEmitMessageOutput) {
                std::cout << rv.mv.time << "," << rv.receiver << "," << rv.sender << ",1.0,0.0\n";
            }
            return;

        case VectorDisposition::Stale:
            if (!silent && kEmitMessageOutput) {
                std::cout << rv.mv.time << "," << rv.receiver << "," << rv.sender << ","
                          << res.confidence << ",0.0\n";
            }
            return;

        case VectorDisposition::Evaluated:
            if (m_minScores.find(rv.sender) == m_minScores.end() || res.confidence < m_minScores[rv.sender]) {
                m_minScores[rv.sender] = res.confidence;
            }
            if (!silent && kEmitMessageOutput) {
                std::cout << rv.mv.time << "," << rv.receiver << "," << rv.sender << ","
                          << res.confidence << "," << res.distError << "\n";
            }
            return;
        }
    }
};

// -----------------------------------------------------------------------------
// Main Runner
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::string drModelName = "HistoryAwareCTRA";
    std::string trustModelName = "BetaReputation";
    std::string dirPath = "";
    bool profileMode = false;
    bool minScoresOnly = false;

    // Every field defaults to the paper-calibrated value from Constants.h;
    // the flags below override individual entries.
    EngineConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dr" && i + 1 < argc) {
            drModelName = argv[++i];
        } else if (arg == "--trust" && i + 1 < argc) {
            trustModelName = argv[++i];
        } else if (arg == "--dir" && i + 1 < argc) {
            dirPath = argv[++i];
        } else if (arg == "--maxPosDev" && i + 1 < argc) {
            cfg.maxPosDev = std::stod(argv[++i]);
        } else if (arg == "--speedWeight" && i + 1 < argc) {
            cfg.contextSpeedWeight = std::stod(argv[++i]);
        } else if (arg == "--yawWeight" && i + 1 < argc) {
            cfg.contextYawWeight = std::stod(argv[++i]);
        } else if (arg == "--priorWeight" && i + 1 < argc) {
            cfg.priorWeight = std::stod(argv[++i]);
        } else if (arg == "--speedTolerance" && i + 1 < argc) {
            cfg.speedToleranceMps = std::stod(argv[++i]);
        } else if (arg == "--alphaReward" && i + 1 < argc) {
            cfg.alphaReward = std::stod(argv[++i]);
        } else if (arg == "--softMarginRatio" && i + 1 < argc) {
            cfg.softMarginRatio = std::stod(argv[++i]);
        } else if (arg == "--penaltySoftBase" && i + 1 < argc) {
            cfg.penaltySoftBase = std::stod(argv[++i]);
        } else if (arg == "--penaltySevereBase" && i + 1 < argc) {
            cfg.penaltySevereBase = std::stod(argv[++i]);
        } else if (arg == "--maxBetaPenalty" && i + 1 < argc) {
            cfg.maxBetaPenalty = std::stod(argv[++i]);
        } else if (arg == "--no-consistency") {
            cfg.enableConsistency = false;
        } else if (arg == "--no-dyn-thresh") {
            cfg.enableDynThresh = false;
        } else if (arg == "--no-ols") {
            cfg.enableOLS = false;
        } else if (arg == "--no-dr") {
            cfg.enablePrediction = false;
        } else if (arg == "--profile") {
            profileMode = true;
        } else if (arg == "--min-scores") {
            minScoresOnly = true;
        } else if ((arg == "--historySize" || arg == "--history-size" || arg == "--history") && i + 1 < argc) {
            cfg.historyWindow = std::stoul(argv[++i]);
        }
    }
    
    if (dirPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " --dir <path> [--dr <model>] [--trust <model>] [--historySize <size>] [--profile] [--min-scores]\n";
        std::cerr << "Ablation flags: [--no-consistency] [--no-dyn-thresh] [--no-ols] [--no-dr]\n";
        return 1;
    }

    // If OLS smoothing is ablated, use bare CTRA model
    if (!cfg.enableOLS && drModelName == "HistoryAwareCTRA") {
        drModelName = "CTRA";
    }

    // 1. Discover all trace files
    auto traceFiles = ParquetTraceReader::getTraceFiles(dirPath);
    if (traceFiles.empty()) {
        std::cerr << "Error: No traceJSON-*.parquet files found in " << dirPath << "\n";
        return 1;
    }

    // 2. Initialize engine
    bool engineSilent = profileMode || minScoresOnly;
    RunnerEngine engine(drModelName, trustModelName, cfg, engineSilent);

    // 3. Process stream
    if (profileMode) {
        std::cout << "Loading " << traceFiles.size() << " trace files for profiling...\n";
        std::vector<std::vector<ReceivedVector>> fileBatches;
        fileBatches.reserve(traceFiles.size());
        size_t totalMessages = 0;
        for (const auto& fpath : traceFiles) {
            auto msgs = loadMotionVectors(fpath);
            totalMessages += msgs.size();
            fileBatches.push_back(std::move(msgs));
        }

        std::cout << "Starting performance profiling over " << totalMessages << " messages (" << traceFiles.size() << " observer nodes)...\n";
        auto start = std::chrono::high_resolution_clock::now();
        
        for (const auto& batch : fileBatches) {
            engine.reset();
            for (const auto& msg : batch) {
                engine.processMessage(msg);
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        
        std::cout << "----------------------------------------\n";
        std::cout << "PROFILING RESULTS\n";
        std::cout << "----------------------------------------\n";
        std::cout << "Observer Files:       " << traceFiles.size() << "\n";
        std::cout << "History Size:         " << cfg.historyWindow << "\n";
        std::cout << "Total Execution Time: " << duration << " us\n";
        std::cout << "Messages Processed:   " << totalMessages << "\n";
        std::cout << "Avg Latency per Msg:  " << (totalMessages > 0 ? (double)duration / totalMessages : 0.0) << " us\n";
        std::cout << "Throughput:           " << (duration > 0 ? (totalMessages * 1000000.0) / duration : 0.0) << " msgs/s\n";
        std::cout << "----------------------------------------\n";
    } else if (minScoresOnly) {
        std::map<long, double> globalMinScores;
        for (const auto& fpath : traceFiles) {
            auto msgs = loadMotionVectors(fpath);
            engine.reset();
            for (const auto& msg : msgs) {
                engine.processMessage(msg);
            }
            for (const auto& [sender, sc] : engine.getMinScores()) {
                if (globalMinScores.find(sender) == globalMinScores.end() || sc < globalMinScores[sender]) {
                    globalMinScores[sender] = sc;
                }
            }
        }
        std::cout << "sender,score\n";
        for (const auto& [sender, sc] : globalMinScores) {
            std::cout << sender << "," << sc << "\n";
        }
    } else {
        std::cout << "rcvTime,receiver,sender,score,distError\n";
        for (const auto& fpath : traceFiles) {
            auto msgs = loadMotionVectors(fpath);
            engine.reset();
            for (const auto& msg : msgs) {
                engine.processMessage(msg);
            }
        }
    }

    return 0;
}
