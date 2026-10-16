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

// Apache Arrow & Parquet
#ifdef USE_ARROW
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#endif

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
// Parquet Message Extractor (Adapted for BSM mapping)
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
#ifdef USE_ARROW
        std::vector<std::string> parquet_files;
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                if (entry.path().extension() == ".parquet") {
                    parquet_files.push_back(entry.path().string());
                }
            }
        } else if (std::filesystem::exists(path)) {
            parquet_files.push_back(path);
        }
        std::sort(parquet_files.begin(), parquet_files.end());

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
            auto receiver_id_col = table->GetColumnByName("receiver_id");
            auto sender_col = table->GetColumnByName("sender");
            auto pos_x_col = table->GetColumnByName("pos_x");
            auto pos_y_col = table->GetColumnByName("pos_y");
            auto pos_nx_col = table->GetColumnByName("pos_noise_x");
            auto pos_ny_col = table->GetColumnByName("pos_noise_y");
            auto spd_x_col = table->GetColumnByName("spd_x");
            auto spd_y_col = table->GetColumnByName("spd_y");
            auto spd_nx_col = table->GetColumnByName("spd_noise_x");
            auto spd_ny_col = table->GetColumnByName("spd_noise_y");
            auto acl_x_col = table->GetColumnByName("acl_x");
            auto acl_y_col = table->GetColumnByName("acl_y");
            auto acl_nx_col = table->GetColumnByName("acl_noise_x");
            auto acl_ny_col = table->GetColumnByName("acl_noise_y");
            auto hed_x_col = table->GetColumnByName("hed_x");
            auto hed_y_col = table->GetColumnByName("hed_y");
            auto hed_nx_col = table->GetColumnByName("hed_noise_x");
            auto hed_ny_col = table->GetColumnByName("hed_noise_y");
            
            auto epos_x_col = table->GetColumnByName("ego_pos_x");
            auto epos_y_col = table->GetColumnByName("ego_pos_y");
            auto ehed_x_col = table->GetColumnByName("ego_hed_x");
            auto ehed_y_col = table->GetColumnByName("ego_hed_y");
            auto espd_x_col = table->GetColumnByName("ego_spd_x");
            auto espd_y_col = table->GetColumnByName("ego_spd_y");
            auto recvp_col  = table->GetColumnByName("receiver_pseudo");
            
            if (!type_col || !rcvTime_col || !sender_col || !pos_x_col || !epos_x_col) continue;
            
            for (int chunk_idx = 0; chunk_idx < type_col->num_chunks(); ++chunk_idx) {
                auto type_arr = std::static_pointer_cast<arrow::Int64Array>(type_col->chunk(chunk_idx));
                auto time_arr = std::static_pointer_cast<arrow::DoubleArray>(rcvTime_col->chunk(chunk_idx));
                auto recvid_arr = std::static_pointer_cast<arrow::Int64Array>(receiver_id_col->chunk(chunk_idx));
                auto sender_arr = std::static_pointer_cast<arrow::Int64Array>(sender_col->chunk(chunk_idx));
                
                auto px_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_x_col->chunk(chunk_idx));
                auto py_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_y_col->chunk(chunk_idx));
                auto pnx_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_nx_col->chunk(chunk_idx));
                auto pny_arr = std::static_pointer_cast<arrow::DoubleArray>(pos_ny_col->chunk(chunk_idx));
                
                auto sx_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_x_col->chunk(chunk_idx));
                auto sy_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_y_col->chunk(chunk_idx));
                auto snx_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_nx_col->chunk(chunk_idx));
                auto sny_arr = std::static_pointer_cast<arrow::DoubleArray>(spd_ny_col->chunk(chunk_idx));
                
                auto ax_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_x_col->chunk(chunk_idx));
                auto ay_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_y_col->chunk(chunk_idx));
                auto anx_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_nx_col->chunk(chunk_idx));
                auto any_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_ny_col->chunk(chunk_idx));
                
                auto hx_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_x_col->chunk(chunk_idx));
                auto hy_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_y_col->chunk(chunk_idx));
                auto hnx_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_nx_col->chunk(chunk_idx));
                auto hny_arr = std::static_pointer_cast<arrow::DoubleArray>(hed_ny_col->chunk(chunk_idx));
                
                auto epx_arr = std::static_pointer_cast<arrow::DoubleArray>(epos_x_col->chunk(chunk_idx));
                auto epy_arr = std::static_pointer_cast<arrow::DoubleArray>(epos_y_col->chunk(chunk_idx));
                auto ehx_arr = std::static_pointer_cast<arrow::DoubleArray>(ehed_x_col->chunk(chunk_idx));
                auto ehy_arr = std::static_pointer_cast<arrow::DoubleArray>(ehed_y_col->chunk(chunk_idx));
                
                std::shared_ptr<arrow::DoubleArray> esx_arr = (espd_x_col && chunk_idx < espd_x_col->num_chunks()) ? std::static_pointer_cast<arrow::DoubleArray>(espd_x_col->chunk(chunk_idx)) : nullptr;
                std::shared_ptr<arrow::DoubleArray> esy_arr = (espd_y_col && chunk_idx < espd_y_col->num_chunks()) ? std::static_pointer_cast<arrow::DoubleArray>(espd_y_col->chunk(chunk_idx)) : nullptr;
                std::shared_ptr<arrow::Int64Array> recvp_arr = (recvp_col && chunk_idx < recvp_col->num_chunks()) ? std::static_pointer_cast<arrow::Int64Array>(recvp_col->chunk(chunk_idx)) : nullptr;
                
                for (int64_t i = 0; i < type_arr->length(); ++i) {
                    if (!type_arr->IsNull(i) && type_arr->Value(i) == 3) {
                        BasicSafetyMessage msg;
                        msg.arrivalTime = time_arr->Value(i);
                        msg.pseudonym = sender_arr->Value(i);
                        msg.receiver_id = recvid_arr->Value(i);
                        
                        msg.pos = veins::Coord(px_arr->Value(i), py_arr->Value(i), 0);
                        msg.posConf = veins::Coord(pnx_arr->Value(i), pny_arr->Value(i), 0);
                        
                        msg.speed = veins::Coord(sx_arr->Value(i), sy_arr->Value(i), 0);
                        msg.speedConf = veins::Coord(snx_arr->Value(i), sny_arr->Value(i), 0);
                        
                        msg.accel = veins::Coord(ax_arr->Value(i), ay_arr->Value(i), 0);
                        msg.accelConf = veins::Coord(anx_arr->Value(i), any_arr->Value(i), 0);
                        
                        msg.heading = veins::Coord(hx_arr->Value(i), hy_arr->Value(i), 0); 
                        msg.headingConf = veins::Coord(hnx_arr->Value(i), hny_arr->Value(i), 0);
                        
                        msg.recv_pos = veins::Coord(epx_arr->Value(i), epy_arr->Value(i), 0);
                        msg.recv_heading = veins::Coord(ehx_arr->Value(i), ehy_arr->Value(i), 0);
                        
                        if (esx_arr && esy_arr && !esx_arr->IsNull(i) && !esy_arr->IsNull(i)) {
                            msg.recv_speed = veins::Coord(esx_arr->Value(i), esy_arr->Value(i), 0);
                        } else {
                            msg.recv_speed = veins::Coord(0, 0, 0);
                        }
                        
                        if (recvp_arr && !recvp_arr->IsNull(i)) {
                            msg.receiver_pseudo = recvp_arr->Value(i);
                        } else {
                            msg.receiver_pseudo = msg.receiver_id;
                        }
                        
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
            
        return all_messages;
#else
        std::cerr << "[!] Error: Built without Apache Arrow support. Please provide a .bin dataset.\n";
        return {};
#endif
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
