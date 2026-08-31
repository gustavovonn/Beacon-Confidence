#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <filesystem>

// Apache Arrow & Parquet
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

// F2MD Dependencies (You must copy these from the veins-f2md repo into an f2md/ folder)
#include "F2MDParameters.h"
#include "mdChecks/LegacyChecks.h"
#include "mdApplications/ThresholdApp.h"
#include "mdBase/NodeTable.h"
#include "mdSupport/networkLinksLib/LinkControl.h"
#include "BasicSafetyMessage.h"

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
struct PrevState {
    double time;
    double speed;
    double heading;
};

// -----------------------------------------------------------------------------
// Parquet Message Extractor (Adapted for BSM mapping)
// -----------------------------------------------------------------------------
class ParquetMessageExtractor {
public:
    static std::vector<BasicSafetyMessage> extractMessages(const std::string& dirPath) {
        std::vector<std::string> parquet_files;
        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (entry.path().extension() == ".parquet" && 
                entry.path().filename().string().find("traceJSON") == 0) {
                parquet_files.push_back(entry.path().string());
            }
        }

        std::vector<BasicSafetyMessage> all_messages;
        
        for (const auto& fpath : parquet_files) {
            auto infile_result = arrow::io::ReadableFile::Open(fpath);
            if (!infile_result.ok()) continue;
            auto infile = infile_result.ValueOrDie();
            
            auto reader_result = parquet::arrow::OpenFile(infile, arrow::default_memory_pool());
            if (!reader_result.ok()) continue;
            auto reader = std::move(reader_result.ValueOrDie());
            
            auto table_result = reader->ReadTable();
            if (!table_result.ok()) continue;
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
            
            if (!type_col || !rcvTime_col || !sender_col || !pos_x_col || !pos_y_col) continue;
            
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
                
                for (int64_t i = 0; i < type_arr->length(); ++i) {
                    if (!type_arr->IsNull(i) && type_arr->Value(i) == 3) {
                        BasicSafetyMessage msg;
                        msg.arrivalTime = time_arr->Value(i);
                        msg.pseudonym = sender_arr->Value(i);
                        msg.pos = veins::Coord(px_arr->Value(i), py_arr->Value(i), 0);
                        
                        double sx = sx_arr->Value(i);
                        double sy = sy_arr->Value(i);
                        double speedMag = std::sqrt(sx*sx + sy*sy);
                        msg.speed = veins::Coord(sx, sy, 0); // F2MD handles speed as a direction vector
                        
                        double hx = hx_arr->Value(i);
                        double hy = hy_arr->Value(i);
                        msg.heading = veins::Coord(hx, hy, 0); 
                        
                        all_messages.push_back(msg);
                    }
                }
            }
        }
        
        // Sort chronologically
        std::sort(all_messages.begin(), all_messages.end(), 
            [](const BasicSafetyMessage& a, const BasicSafetyMessage& b) {
                return a.arrivalTime < b.arrivalTime;
            });
            
        computeDerivedKinematics(all_messages);
        return all_messages;
    }

private:
    static void computeDerivedKinematics(std::vector<BasicSafetyMessage>& msgs) {
        std::map<long, PrevState> prev_states;
        
        for (auto& msg : msgs) {
            double currentSpeedMag = std::sqrt(msg.speed.x*msg.speed.x + msg.speed.y*msg.speed.y);
            double currentHeadingDeg = std::atan2(msg.heading.x, msg.heading.y) * 180.0 / M_PI;
            if (currentHeadingDeg < 0) currentHeadingDeg += 360.0;

            auto p_it = prev_states.find(msg.pseudonym);
            if (p_it != prev_states.end()) {
                double dt = msg.arrivalTime - p_it->second.time;
                if (dt <= 0.0) dt = 0.1;
                
                double accelMag = (currentSpeedMag - p_it->second.speed) / dt;
                // Simplified accel vector in direction of heading for F2MD
                msg.accel = veins::Coord(msg.heading.x * accelMag, msg.heading.y * accelMag, 0);
            } else {
                msg.accel = veins::Coord(0,0,0);
            }
            prev_states[msg.pseudonym] = {msg.arrivalTime, currentSpeedMag, currentHeadingDeg};
        }
    }
};

// -----------------------------------------------------------------------------
// F2MD Evaluation Engine
// -----------------------------------------------------------------------------
class F2MDEvaluationEngine {
private:
    F2MDParameters params;
    NodeTable detectedNodes;
    std::unordered_map<LAddress::L2Type, veins::Coord> realDynamicMap;
    LinkControl linkControl;
    ThresholdApp app;
    bool silent;

    // "Observer" node static context
    veins::Coord myPosition = {0, 0, 0};
    veins::Coord mySpeed    = {0, 0, 0};
    veins::Coord myHeading  = {1, 0, 0};
    veins::Coord mySize     = {2.0, 4.5, 0};
    LAddress::L2Type myId   = 0;

public:
    F2MDEvaluationEngine(bool silentMode = false) : app(1, 0.28125), silent(silentMode) {
        linkControl.initialize(&params, nullptr);
        if (!silent) {
            std::cout << "rcvTime,sender,flagged\n";
        }
    }
    
    void processMessage(BasicSafetyMessage& bsm) {
        // Update global ground truth map so ProximityPlausibilityCheck doesn't always pass
        realDynamicMap[bsm.getSenderPseudonym()] = bsm.getSenderPos();

        veins::Coord myLimits = {params.MIN_MAX_SPEED, params.MIN_MAX_ACCEL, params.MIN_MAX_DECEL};
        
        // Stage 1: Run Plausibility Checks
        LegacyChecks checker(
            1, 0, myPosition, mySpeed, myHeading, mySize, myLimits,
            &linkControl, &realDynamicMap, myId, &params
        );

        BsmCheck bsmCheck = checker.CheckBSM(&bsm, &detectedNodes);

        // Stage 2: Decision Application
        bool flagged = app.CheckNodeForReport(0, &bsm, &bsmCheck, &detectedNodes);

        // Update NodeTable History and Kalman Filters for next iteration
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

        if (!silent) {
            // std::cout << bsm.getArrivalTime() << "," << bsm.getSenderPseudonym() << "," << (flagged ? 1 : 0) << "\n";
            std::cout << bsm.getArrivalTime().dbl() << "," << bsm.getSenderPseudonym() << "," << (flagged ? 1 : 0) << "\n";
        }
    }
};

// -----------------------------------------------------------------------------
// Main Runner
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::string dirPath = "";
    bool profileMode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dir" && i + 1 < argc) {
            dirPath = argv[++i];
        } else if (arg == "--profile") {
            profileMode = true;
        }
    }
    
    if (dirPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " --dir <path> [--profile]\n";
        return 1;
    }

    // 1. Extract and format messages
    auto messages = ParquetMessageExtractor::extractMessages(dirPath);
    if (messages.empty()) {
        std::cerr << "No valid messages found in " << dirPath << "\n";
        return 1;
    }
    
    // 2. Initialize F2MD engine
    F2MDEvaluationEngine engine(profileMode);

    // 3. Process stream
    if (profileMode) {
        std::cout << "Starting F2MD evaluation over " << messages.size() << " messages...\n";
        auto start = std::chrono::high_resolution_clock::now();
        
        for (auto& msg : messages) {
            engine.processMessage(msg);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        
        std::cout << "----------------------------------------\n";
        std::cout << "Total Execution Time: " << duration << " us\n";
        std::cout << "Messages Processed:   " << messages.size() << "\n";
        std::cout << "Avg Latency per Msg:  " << (double)duration / messages.size() << " us\n";
        std::cout << "----------------------------------------\n";
    } else {
        for (auto& msg : messages) {
            engine.processMessage(msg);
        }
    }

    return 0;
}