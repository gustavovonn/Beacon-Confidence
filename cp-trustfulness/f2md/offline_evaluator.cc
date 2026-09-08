#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <cmath>
#include <memory>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>

#ifdef USE_ARROW
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#endif
#include "BinaryMessageFormat.h"

// Trustfulness (OLS-CTRA) Dependencies
#include "IDeadReckoningModel.h"
#include "TrustEvaluators.h"
#include "VeremiEngine.h"

// F2MD Dependencies
#include "mdBase/BsmCheck.h"
#include "mdChecks/LegacyChecks.h"
#include "mdChecks/CaTChChecks.h"
#include "mdChecks/ExperiChecks.h"
#include "mdSupport/networkLinksLib/LinkControl.h"
#include "BasicSafetyMessage.h"
#include "F2MDEngine.h"

struct BenchmarkResult {
    std::string dataset;
    std::string tier;
    std::string stage1;
    std::string stage2;
    int total_messages = 0;
    double stage1_latency_us = 0.0;
    double stage2_latency_us = 0.0;
    double total_latency_us = 0.0;
    double throughput_msgs_sec = 0.0;
    double total_time_ms = 0.0;
};

void printBenchmarkResult(const BenchmarkResult& res) {
    std::cout << "\n--- COMPUTATIONAL COST PROFILING ---\n";
    if (!res.dataset.empty()) {
        std::cout << "Dataset                  : " << res.dataset << "\n";
    }
    std::cout << "Tier                     : " << res.tier << "\n";
    std::cout << "Stage 1 (Checker)        : " << res.stage1 << "\n";
    std::cout << "Stage 2 (Scorer/App)     : " << res.stage2 << "\n";
    std::cout << "Total Messages Processed : " << res.total_messages << "\n";
    if (res.stage1_latency_us > 0.0) {
        std::cout << "Stage 1 Latency (us/msg) : " << std::fixed << std::setprecision(4) << res.stage1_latency_us << "\n";
    }
    if (res.stage2_latency_us > 0.0) {
        std::cout << "Stage 2 Latency (us/msg) : " << std::fixed << std::setprecision(4) << res.stage2_latency_us << "\n";
    }
    std::cout << "Total Latency (us/msg)   : " << std::fixed << std::setprecision(4) << res.total_latency_us << "\n";
    std::cout << "Pipeline Throughput      : " << std::scientific << res.throughput_msgs_sec << " msgs/sec\n";
    std::cout << "Total Execution Time (ms): " << std::fixed << std::setprecision(3) << res.total_time_ms << " ms\n";
}

void appendCsvResult(const std::string& csvPath, const BenchmarkResult& res) {
    if (csvPath.empty()) return;
    bool writeHeader = !std::filesystem::exists(csvPath);
    std::ofstream out(csvPath, std::ios::app);
    if (!out.is_open()) return;

    if (writeHeader) {
        out << "Dataset,Tier,Stage1,Stage2,Messages,Stage1_Latency_us,Stage2_Latency_us,Total_Latency_us,Throughput_msgs_sec,Total_Time_ms\n";
    }
    out << res.dataset << ","
        << res.tier << ","
        << res.stage1 << ","
        << res.stage2 << ","
        << res.total_messages << ","
        << std::fixed << std::setprecision(5) << res.stage1_latency_us << ","
        << res.stage2_latency_us << ","
        << res.total_latency_us << ","
        << std::fixed << std::setprecision(2) << res.throughput_msgs_sec << ","
        << std::fixed << std::setprecision(3) << res.total_time_ms << "\n";
}

// -----------------------------------------------------------------------------
// Benchmark Runner Functions
// -----------------------------------------------------------------------------

BenchmarkResult runOlsCtraPipeline(const std::string& dataPath, const std::string& tier, const std::string& stage2, int maxMsgs, bool profile) {
    BenchmarkResult res;
    res.dataset = std::filesystem::path(dataPath).stem().string();
    res.tier = tier;
    res.stage1 = "ols-ctra";
    res.stage2 = stage2;

    double maxPosDev = 4.3506;
    double speedWeight = 5.2264;
    double yawWeight = 0.6536;

    auto all_messages = ParquetMessageExtractor::extractMessages(dataPath);
    if (all_messages.empty()) {
        std::cerr << "No messages extracted from: " << dataPath << "\n";
        return res;
    }

    size_t limit = (maxMsgs > 0 && maxMsgs < (int)all_messages.size()) ? maxMsgs : all_messages.size();
    res.total_messages = (int)limit;

    TrustEvaluationEngine engine("HistoryAwareCTRA", "BetaReputation", maxPosDev, speedWeight, yawWeight, true);

    double sum_s1_us = 0.0;
    double sum_s2_us = 0.0;

    auto start_time = std::chrono::high_resolution_clock::now();

    if (tier == "1") {
        for (size_t i = 0; i < limit; ++i) {
            double s1 = 0.0;
            engine.runStage1(all_messages[i], &s1);
            sum_s1_us += s1;
        }
    } else if (tier == "2") {
        for (size_t i = 0; i < limit; ++i) {
            double s2 = 0.0;
            engine.runStage2(all_messages[i], 1.5, 0.5, &s2);
            sum_s2_us += s2;
        }
    } else {
        // Tier 3 or all
        for (size_t i = 0; i < limit; ++i) {
            double s1 = 0.0, s2 = 0.0;
            engine.processMessage(all_messages[i], &s1, &s2);
            sum_s1_us += s1;
            sum_s2_us += s2;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_us = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    res.total_time_ms = duration_us / 1000.0;
    res.stage1_latency_us = sum_s1_us / (double)limit;
    res.stage2_latency_us = sum_s2_us / (double)limit;

    if (tier == "1") {
        res.total_latency_us = res.stage1_latency_us;
    } else if (tier == "2") {
        res.total_latency_us = res.stage2_latency_us;
    } else {
        res.total_latency_us = res.stage1_latency_us + res.stage2_latency_us;
    }
    res.throughput_msgs_sec = (res.total_latency_us > 0.0) ? (1000000.0 / res.total_latency_us) : 0.0;

    return res;
}

BenchmarkResult runF2mdPipeline(const std::string& dataPath, const std::string& tier, const std::string& stage1, const std::string& stage2, int maxMsgs, bool profile) {
    BenchmarkResult res;
    res.dataset = std::filesystem::path(dataPath).stem().string();
    res.tier = tier;
    res.stage1 = stage1;
    res.stage2 = stage2;

    auto all_messages = F2MDParquetExtractor::extractMessages(dataPath);
    if (all_messages.empty()) {
        std::cerr << "No BSM messages extracted from: " << dataPath << "\n";
        return res;
    }

    size_t limit = (maxMsgs > 0 && maxMsgs < (int)all_messages.size()) ? maxMsgs : all_messages.size();
    res.total_messages = (int)limit;

    LinkControl globalLinkControl;
    std::unordered_map<long, std::unique_ptr<F2MDEvaluationEngine>> engines;

    double sum_s1_us = 0.0;
    double sum_s2_us = 0.0;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < limit; ++i) {
        auto& msg = all_messages[i];
        if (engines.find(msg.receiver_id) == engines.end()) {
            engines[msg.receiver_id] = std::make_unique<F2MDEvaluationEngine>(&globalLinkControl, stage1, stage2, true, "");
        }

        if (tier == "1") {
            double s1 = 0.0;
            BsmCheck chk = engines[msg.receiver_id]->runStage1(msg, &s1);
            engines[msg.receiver_id]->updateHistory(msg, chk);
            sum_s1_us += s1;
        } else if (tier == "2") {
            BsmCheck chk;
            chk.setSpeedPlausibility(0.95);
            chk.setPositionConsistancy(0.92);
            chk.setPositionSpeedConsistancy(0.90);
            chk.setRangePlausibility(0.95);
            chk.setPositionPlausibility(0.95);
            engines[msg.receiver_id]->updateHistory(msg, chk);
            double s2 = 0.0;
            engines[msg.receiver_id]->runStage2(msg, chk, &s2);
            sum_s2_us += s2;
        } else {
            // Tier 3
            double s1 = 0.0, s2 = 0.0;
            engines[msg.receiver_id]->processMessage(msg, &s1, &s2);
            sum_s1_us += s1;
            sum_s2_us += s2;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_us = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    res.total_time_ms = duration_us / 1000.0;
    res.stage1_latency_us = sum_s1_us / (double)limit;
    res.stage2_latency_us = sum_s2_us / (double)limit;

    if (tier == "1") {
        res.total_latency_us = res.stage1_latency_us;
    } else if (tier == "2") {
        res.total_latency_us = res.stage2_latency_us;
    } else {
        res.total_latency_us = res.stage1_latency_us + res.stage2_latency_us;
    }
    res.throughput_msgs_sec = (res.total_latency_us > 0.0) ? (1000000.0 / res.total_latency_us) : 0.0;

    return res;
}

// -----------------------------------------------------------------------------
// Main Unified Runner
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::string tier = "3";
    std::string stage1 = "ols-ctra";
    std::string stage2 = "beta-reputation";
    std::string modelName = "";
    std::string dataPath = "";
    std::string csvOut = "";
    int maxMsgs = 0;
    int repeat = 1;
    bool profileMode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--tier" && i + 1 < argc) {
            tier = argv[++i];
        } else if (arg == "--stage1" && i + 1 < argc) {
            stage1 = argv[++i];
        } else if (arg == "--stage2" && i + 1 < argc) {
            stage2 = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            modelName = argv[++i];
        } else if ((arg == "--dir" || arg == "--file") && i + 1 < argc) {
            dataPath = argv[++i];
        } else if (arg == "--max-msgs" && i + 1 < argc) {
            maxMsgs = std::stoi(argv[++i]);
        } else if (arg == "--repeat" && i + 1 < argc) {
            repeat = std::stoi(argv[++i]);
        } else if (arg == "--csv-out" && i + 1 < argc) {
            csvOut = argv[++i];
        } else if (arg == "--profile") {
            profileMode = true;
        }
    }

    // Preset mappings for backward compatibility
    if (!modelName.empty()) {
        if (modelName == "ols-ctra") {
            stage1 = "ols-ctra";
            stage2 = "beta-reputation";
        } else if (modelName == "f2md-legacy") {
            stage1 = "f2md-legacy";
            stage2 = "threshold";
        } else if (modelName == "f2md-catch") {
            stage1 = "f2md-catch";
            stage2 = "threshold";
        } else if (modelName == "f2md-experi") {
            stage1 = "f2md-experi";
            stage2 = "threshold";
        } else if (modelName == "f2md-legacy-behavioral") {
            stage1 = "f2md-legacy";
            stage2 = "behavioral";
        } else if (modelName == "f2md-catch-behavioral") {
            stage1 = "f2md-catch";
            stage2 = "behavioral";
        } else if (modelName == "f2md-catch-ml") {
            std::cerr << "[!] Error: MachineLearningApp is not implemented yet (pending manual validation).\n";
            return 1;
        } else {
            stage1 = modelName;
        }
    }

    if (stage2 == "ml" || stage2 == "ml-app" || stage2 == "mlp") {
        std::cerr << "[!] Error: MachineLearningApp is not implemented yet (pending manual validation).\n";
        return 1;
    }

    if (dataPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " [--tier <1|2|3|all>] [--stage1 <ols-ctra|legacy|catch|experi>]"
                  << " [--stage2 <beta-reputation|threshold|behavioral|aggregation|experi|ml>]"
                  << " [--model <preset>] --dir <path> [--max-msgs <N>] [--profile] [--csv-out <path>] [--repeat <N>]\n";
        return 1;
    }

    if (!std::filesystem::exists(dataPath)) {
        std::cerr << "[!] Error: Dataset path does not exist: " << dataPath << "\n";
        return 1;
    }

    std::vector<std::string> tiersToRun;
    if (tier == "all") {
        tiersToRun = {"1", "2", "3"};
    } else {
        tiersToRun = {tier};
    }

    for (const auto& curTier : tiersToRun) {
        for (int r = 0; r < repeat; ++r) {
            BenchmarkResult res;
            if (stage1 == "ols-ctra") {
                res = runOlsCtraPipeline(dataPath, curTier, stage2, maxMsgs, profileMode);
            } else {
                res = runF2mdPipeline(dataPath, curTier, stage1, stage2, maxMsgs, profileMode);
            }

            if (res.total_messages <= 0) {
                std::cerr << "[!] Error: 0 messages processed from dataset: " << dataPath << "\n";
                return 1;
            }

            if (profileMode) {
                printBenchmarkResult(res);
            }

            if (!csvOut.empty()) {
                appendCsvResult(csvOut, res);
            }
        }
    }

    return 0;
}
