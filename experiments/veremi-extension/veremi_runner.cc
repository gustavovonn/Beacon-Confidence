#include "../include/IDeadReckoningModel.h"
#include "../include/DeadReckoningModels.h"
#include "../include/CoordinatePolicies.h"
#include "../include/PhysicalInvariance.h"
#include "../include/CheckerPipeline.h"
#include "../include/TrustEvaluators.h"
#include "../include/Constants.h"
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

struct RawMessage {
    double rcvTime;
    long receiver;
    long sender;
    double pos_x;
    double pos_y;
    double speed;
    double heading;
    double accel;     // Parsed longitudinal acceleration from dataset (or 0.0)
    double yawRate;   // Yaw rate (0.0 if not logged in dataset)
    
    // For sorting
    bool operator<(const RawMessage& other) const {
        return rcvTime < other.rcvTime;
    }
};

// -----------------------------------------------------------------------------
// Parquet Message Extractor
// -----------------------------------------------------------------------------
class ParquetMessageExtractor {
public:
    static long extractReceiverIdFromPath(const std::string& fpath) {
        std::string fname = std::filesystem::path(fpath).filename().string();
        size_t p1 = fname.find('-');
        if (p1 != std::string::npos) {
            size_t p2 = fname.find('-', p1 + 1);
            if (p2 != std::string::npos) {
                try {
                    return std::stol(fname.substr(p1 + 1, p2 - p1 - 1));
                } catch (...) {}
            }
        }
        return -1;
    }

    static std::vector<std::string> getTraceFiles(const std::string& dirPath) {
        std::vector<std::string> parquet_files;
        if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
            return parquet_files;
        }
        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (entry.path().extension() == ".parquet") {
                if (entry.path().filename().string().find("traceJSON") == 0) {
                    parquet_files.push_back(entry.path().string());
                }
            }
        }
        std::sort(parquet_files.begin(), parquet_files.end());
        return parquet_files;
    }

    static std::vector<RawMessage> extractMessagesFromFile(const std::string& fpath) {
        long receiver_id = extractReceiverIdFromPath(fpath);
        std::vector<RawMessage> messages;

        auto infile_result = arrow::io::ReadableFile::Open(fpath);
        if (!infile_result.ok()) return messages;
        auto infile = infile_result.ValueOrDie();
        
        auto reader_result = parquet::arrow::OpenFile(infile, arrow::default_memory_pool());
        if (!reader_result.ok()) return messages;
        auto reader = std::move(reader_result.ValueOrDie());
        
        auto table_result = reader->ReadTable();
        if (!table_result.ok()) return messages;
        auto table = table_result.ValueOrDie();
        
        auto type_col = table->GetColumnByName("type");
        auto rcvTime_col = table->GetColumnByName("rcvTime");
        auto sender_col = table->GetColumnByName("sender");
        auto pos_x_col = table->GetColumnByName("pos_x");
        auto pos_y_col = table->GetColumnByName("pos_y");
        auto spd_x_col = table->GetColumnByName("spd_x");
        auto spd_y_col = table->GetColumnByName("spd_y");
        auto hed_x_col = table->GetColumnByName("hed_x");
        auto hed_y_col = table->GetColumnByName("hed_y");
        
        auto acl_x_col = table->GetColumnByName("acl_x");
        auto acl_y_col = table->GetColumnByName("acl_y");
        
        if (!type_col || !rcvTime_col || !sender_col || !pos_x_col || !pos_y_col || !spd_x_col || !spd_y_col || !hed_x_col || !hed_y_col) {
            return messages;
        }
        
        for (int chunk_idx = 0; chunk_idx < type_col->num_chunks(); ++chunk_idx) {
            auto type_arr = std::static_pointer_cast<arrow::Int64Array>(type_col->chunk(chunk_idx));
            auto time_arr = std::static_pointer_cast<arrow::DoubleArray>(rcvTime_col->chunk(chunk_idx));
            auto sender_arr = std::static_pointer_cast<arrow::Int64Array>(sender_col->chunk(chunk_idx));
            auto px_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_x_col->chunk(chunk_idx));
            auto py_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_y_col->chunk(chunk_idx));
            auto sx_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_x_col->chunk(chunk_idx));
            auto sy_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_y_col->chunk(chunk_idx));
            auto hx_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_x_col->chunk(chunk_idx));
            auto hy_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_y_col->chunk(chunk_idx));

            std::shared_ptr<arrow::DoubleArray> ax_arr = nullptr;
            std::shared_ptr<arrow::DoubleArray> ay_arr = nullptr;
            if (acl_x_col && acl_y_col && chunk_idx < acl_x_col->num_chunks() && chunk_idx < acl_y_col->num_chunks()) {
                ax_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_x_col->chunk(chunk_idx));
                ay_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_y_col->chunk(chunk_idx));
            }
            
            for (int64_t i = 0; i < type_arr->length(); ++i) {
                if (!type_arr->IsNull(i) && type_arr->Value(i) == 3) {
                    RawMessage msg;
                    msg.rcvTime = time_arr->Value(i);
                    msg.receiver = receiver_id;
                    msg.sender = sender_arr->Value(i);
                    msg.pos_x = px_arr->Value(i);
                    msg.pos_y = py_arr->Value(i);
                    
                    double sx = sx_arr->Value(i);
                    double sy = sy_arr->Value(i);
                    msg.speed = std::sqrt(sx*sx + sy*sy);
                    
                    double hx = hx_arr->Value(i);
                    double hy = hy_arr->Value(i);
                    double h_deg = std::atan2(hx, hy) * 180.0 / M_PI;
                    if (h_deg < 0) h_deg += 360.0;
                    msg.heading = h_deg;

                    // Parse instantaneous longitudinal acceleration if present in dataset
                    if (ax_arr && ay_arr && !ax_arr->IsNull(i) && !ay_arr->IsNull(i)) {
                        double ax = ax_arr->Value(i);
                        double ay = ay_arr->Value(i);
                        double h_rad = h_deg * M_PI / 180.0;
                        // Project 2D Cartesian acceleration onto vehicle heading vector
                        msg.accel = ax * std::sin(h_rad) + ay * std::cos(h_rad);
                    } else {
                        msg.accel = 0.0;
                    }
                    
                    msg.yawRate = 0.0; // Yaw rate is not logged in VeReMi traces
                    
                    messages.push_back(msg);
                }
            }
        }
        
        std::sort(messages.begin(), messages.end());
        
        return messages;
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
    AblationConfig ablation;
    bool silent;
    size_t historySize;
    
    std::map<long, std::deque<KinematicState>> m_cache;
    std::map<long, double> m_scores;
    std::map<long, double> m_minScores;

public:
    TrustEvaluationEngine(const std::string& drModelName, const std::string& trustModelName, 
                          const TrustEvaluationParams& evalParams,
                          bool silentMode = false, size_t histSize = ConstantsApp::DEFAULT_HISTORY_WINDOW,
                          const AblationConfig& ablationConfig = {}) 
        : params(evalParams), ablation(ablationConfig), silent(silentMode), historySize(histSize) {
        drModel = DeadReckoningModelFactory::create(drModelName, "cartesian");
        trustEvaluator = TrustEvaluatorFactory::create(trustModelName, histSize);
    }

    void reset() {
        m_cache.clear();
        m_scores.clear();
        m_minScores.clear();
        if (trustEvaluator) {
            trustEvaluator->reset();
        }
    }

    const std::map<long, double>& getMinScores() const {
        return m_minScores;
    }
    
    void processMessage(const RawMessage& msg) {
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
            if (m_minScores.find(msg.sender) == m_minScores.end()) {
                m_minScores[msg.sender] = 1.0;
            }
            std::deque<KinematicState> newHistory;
            newHistory.push_back(currentState);
            m_cache[msg.sender] = newHistory;
            if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.receiver << "," << msg.sender << ",1.0,0.0\n";
#endif
            }
            return;
        }

        auto& history = it->second;
        KinematicState pastState = history.back();
        
        double dt = currentState.time - pastState.time;
        if (dt < 0.05) return;
        if (dt > ConstantsPhysics::MAX_INTER_FRAME_TIME) {
            history.clear();
            history.push_back(currentState);
            if (!silent) {
#ifndef DISABLE_OUTPUT
                std::cout << msg.rcvTime << "," << msg.receiver << "," << msg.sender << "," 
                          << m_scores[msg.sender] << ",0.0\n";
#endif
            }
            return;
        }

        // 1. Phase 0: Hard Physical Invariance Verification
        PhysicalInvarianceResult physicsResult = phase0_PhysicalInvariance<CartesianCoordinatePolicy>(
            pastState, currentState, params.speedToleranceMps, ablation.enablePhase0);

        // 2. Phase 1: Kinematic Consistency (OLS-CTRA Dead-Reckoning & Dynamic Threshold)
        double distError = 0.0;
        double dynamicMaxPosDev = 0.0;
        phase1_KinematicConsistency(
            drModel.get(), history, pastState, currentState, dt, params, physicsResult,
            distError, dynamicMaxPosDev, ablation.enableDynThresh, ablation.enableDR);

        // 3. Phase 2: Scorer Evidence Dispatch & Posterior Update
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

        double newScore = trustEvaluator->evaluateTrust(context, params);
        m_scores[msg.sender] = newScore;
        if (m_minScores.find(msg.sender) == m_minScores.end() || newScore < m_minScores[msg.sender]) {
            m_minScores[msg.sender] = newScore;
        }

        if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.receiver << "," << msg.sender << "," << newScore << "," 
                      << distError << "\n";
#endif
        }

        history.push_back(currentState);
        while (history.size() > historySize) {
            history.pop_front();
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
    size_t historySize = ConstantsApp::DEFAULT_HISTORY_WINDOW;

    TrustEvaluationParams params;
    AblationConfig ablation;

    // Offline optimized baseline weights:
    params.maxPosDev = 4.3506;
    params.contextSpeedWeight = 5.2264;
    params.contextYawWeight = 0.6536;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dr" && i + 1 < argc) {
            drModelName = argv[++i];
        } else if (arg == "--trust" && i + 1 < argc) {
            trustModelName = argv[++i];
        } else if (arg == "--dir" && i + 1 < argc) {
            dirPath = argv[++i];
        } else if (arg == "--maxPosDev" && i + 1 < argc) {
            params.maxPosDev = std::stod(argv[++i]);
        } else if (arg == "--speedWeight" && i + 1 < argc) {
            params.contextSpeedWeight = std::stod(argv[++i]);
        } else if (arg == "--yawWeight" && i + 1 < argc) {
            params.contextYawWeight = std::stod(argv[++i]);
        } else if (arg == "--priorWeight" && i + 1 < argc) {
            params.priorWeight = std::stod(argv[++i]);
        } else if (arg == "--speedTolerance" && i + 1 < argc) {
            params.speedToleranceMps = std::stod(argv[++i]);
        } else if (arg == "--alphaReward" && i + 1 < argc) {
            params.alphaReward = std::stod(argv[++i]);
        } else if (arg == "--softMarginRatio" && i + 1 < argc) {
            params.softMarginRatio = std::stod(argv[++i]);
        } else if (arg == "--penaltySoftBase" && i + 1 < argc) {
            params.penaltySoftBase = std::stod(argv[++i]);
        } else if (arg == "--penaltySevereBase" && i + 1 < argc) {
            params.penaltySevereBase = std::stod(argv[++i]);
        } else if (arg == "--maxBetaPenalty" && i + 1 < argc) {
            params.maxBetaPenalty = std::stod(argv[++i]);
        } else if (arg == "--no-phase0") {
            ablation.enablePhase0 = false;
        } else if (arg == "--no-dyn-thresh") {
            ablation.enableDynThresh = false;
        } else if (arg == "--no-ols") {
            ablation.enableOLS = false;
        } else if (arg == "--no-dr") {
            ablation.enableDR = false;
        } else if (arg == "--profile") {
            profileMode = true;
        } else if (arg == "--min-scores") {
            minScoresOnly = true;
        } else if ((arg == "--historySize" || arg == "--history-size" || arg == "--history") && i + 1 < argc) {
            historySize = std::stoul(argv[++i]);
        }
    }
    
    if (dirPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " --dir <path> [--dr <model>] [--trust <model>] [--historySize <size>] [--profile] [--min-scores]\n";
        std::cerr << "Ablation flags: [--no-phase0] [--no-dyn-thresh] [--no-ols] [--no-dr]\n";
        return 1;
    }

    // If OLS smoothing is ablated, use bare CTRA model
    if (!ablation.enableOLS && drModelName == "HistoryAwareCTRA") {
        drModelName = "CTRA";
    }

    // 1. Discover all trace files
    auto traceFiles = ParquetMessageExtractor::getTraceFiles(dirPath);
    if (traceFiles.empty()) {
        std::cerr << "Error: No traceJSON-*.parquet files found in " << dirPath << "\n";
        return 1;
    }

    // 2. Initialize engine
    bool engineSilent = profileMode || minScoresOnly;
    TrustEvaluationEngine engine(drModelName, trustModelName, params, engineSilent, historySize, ablation);

    // 3. Process stream
    if (profileMode) {
        std::cout << "Loading " << traceFiles.size() << " trace files for profiling...\n";
        std::vector<std::vector<RawMessage>> fileBatches;
        fileBatches.reserve(traceFiles.size());
        size_t totalMessages = 0;
        for (const auto& fpath : traceFiles) {
            auto msgs = ParquetMessageExtractor::extractMessagesFromFile(fpath);
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
        std::cout << "History Size:         " << historySize << "\n";
        std::cout << "Total Execution Time: " << duration << " us\n";
        std::cout << "Messages Processed:   " << totalMessages << "\n";
        std::cout << "Avg Latency per Msg:  " << (totalMessages > 0 ? (double)duration / totalMessages : 0.0) << " us\n";
        std::cout << "Throughput:           " << (duration > 0 ? (totalMessages * 1000000.0) / duration : 0.0) << " msgs/s\n";
        std::cout << "----------------------------------------\n";
    } else if (minScoresOnly) {
        std::map<long, double> globalMinScores;
        for (const auto& fpath : traceFiles) {
            auto msgs = ParquetMessageExtractor::extractMessagesFromFile(fpath);
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
            auto msgs = ParquetMessageExtractor::extractMessagesFromFile(fpath);
            engine.reset();
            for (const auto& msg : msgs) {
                engine.processMessage(msg);
            }
        }
    }

    return 0;
}
