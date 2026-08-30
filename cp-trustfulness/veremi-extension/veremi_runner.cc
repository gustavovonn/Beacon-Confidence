#include "../include/IDeadReckoningModel.h"
#include "../include/TrustEvaluators.h"
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

using namespace artery;

// -----------------------------------------------------------------------------
// Cartesian CTRA Integration
// -----------------------------------------------------------------------------
static void cartesianCtraIntegrate(double startX, double startY, double startSpeed, double startHeading, 
                                   double a, double w, double dt, PredictedState& nextState)
{
    const double headingRad = startHeading * M_PI / 180.0;
    
    if (std::abs(w) < 1e-4)
    {
        const double predictedSpeed = startSpeed + a * dt;
        double dist = startSpeed * dt + 0.5 * a * dt * dt;
        if (predictedSpeed <= 0.0 || dist < 0.0) {
            dist = 0.0;
            nextState.speed = 0.0;
        } else {
            nextState.speed = predictedSpeed;
        }
        
        const double dy = dist * std::cos(headingRad);
        const double dx = dist * std::sin(headingRad);
        
        nextState.lat = startY + dy; // Store Y in lat
        nextState.lon = startX + dx; // Store X in lon
        nextState.heading = startHeading;
        return;
    }
    
    const double v_dt = startSpeed + a * dt;
    const double newHeadingRad = headingRad + w * dt;
    
    const double inv_w = 1.0 / w;
    const double a_over_w2 = a / (w * w);
    
    const double sinH0 = std::sin(headingRad);
    const double cosH0 = std::cos(headingRad);
    const double sinH1 = std::sin(newHeadingRad);
    const double cosH1 = std::cos(newHeadingRad);
    
    const double dx = inv_w * (v_dt * sinH1 - startSpeed * sinH0) + a_over_w2 * (cosH1 - cosH0);
    const double dy = inv_w * (-v_dt * cosH1 + startSpeed * cosH0) + a_over_w2 * (sinH1 - sinH0);
    
    nextState.lat = startY + dy;
    nextState.lon = startX + dx;
    nextState.speed = std::max(0.0, v_dt);
    
    double headingDeg = std::fmod(newHeadingRad * 180.0 / M_PI, 360.0);
    if (headingDeg < 0.0) headingDeg += 360.0;
    nextState.heading = headingDeg;
}

// -----------------------------------------------------------------------------
// Cartesian Dead Reckoning Models
// -----------------------------------------------------------------------------
class CartesianCtraDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(const std::deque<KinematicState>& history, const KinematicState& lastState, PredictedState& nextState, double dt) override {
        if (dt <= 0.0) return;
        cartesianCtraIntegrate(lastState.lon, lastState.lat, lastState.speed, lastState.heading, 
                               lastState.accel, lastState.yawRate, dt, nextState);
    }
    double computeError(const PredictedState& expected, const KinematicState& actual) override {
        double dx = expected.lon - actual.lon;
        double dy = expected.lat - actual.lat;
        return std::sqrt(dx*dx + dy*dy);
    }
    std::string getName() const override { return "CartesianCTRA"; }
};

class CartesianHistoryAwareCtraModel : public IDeadReckoningModel
{
private:
    double unwrapHeading(double currentHeading, double previousHeading) {
        double diff = currentHeading - previousHeading;
        diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
        return previousHeading + diff;
    }
public:
    void predict(const std::deque<KinematicState>& history, const KinematicState& lastState, PredictedState& nextState, double dt) override {
        if (dt <= 0.0) return;
        if (history.size() < 3) {
            CartesianCtraDeadReckoningModel fallback;
            fallback.predict(history, lastState, nextState, dt);
            return;
        }

        double sum_t = 0.0, sum_v = 0.0, sum_h = 0.0;
        double sum_t2 = 0.0, sum_tv = 0.0, sum_th = 0.0;
        const double t0 = history.front().time;
        double prevHeading = history.front().heading;
        const size_t n = history.size();

        for (size_t i = 0; i < n; ++i) {
            const double t = history[i].time - t0;
            const double v = history[i].speed;
            double h = history[i].heading;
            if (i > 0) {
                h = unwrapHeading(h, prevHeading);
                prevHeading = h;
            }
            sum_t  += t;
            sum_v  += v;
            sum_h  += h;
            sum_t2 += t * t;
            sum_tv += t * v;
            sum_th += t * h;
        }

        const double denom = static_cast<double>(n) * sum_t2 - sum_t * sum_t;
        double a, w;
        if (std::abs(denom) < 1e-6) {
            a = lastState.accel;
            w = lastState.yawRate;
        } else {
            a = (static_cast<double>(n) * sum_tv - sum_t * sum_v) / denom;
            const double w_deg = (static_cast<double>(n) * sum_th - sum_t * sum_h) / denom;
            w = w_deg * M_PI / 180.0;
        }

        cartesianCtraIntegrate(lastState.lon, lastState.lat, lastState.speed, lastState.heading, 
                               a, w, dt, nextState);
    }
    double computeError(const PredictedState& expected, const KinematicState& actual) override {
        double dx = expected.lon - actual.lon;
        double dy = expected.lat - actual.lat;
        return std::sqrt(dx*dx + dy*dy);
    }
    std::string getName() const override { return "CartesianHistoryAwareCTRA"; }
};

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------
double calculateCentroidDeviation(const std::deque<KinematicState>& state_history, double receivedLat, double receivedLon)
{
    if (state_history.empty()) return 0.0;
    double sumLat = 0.0, sumLon = 0.0;
    for (const auto& state : state_history) {
        sumLat += state.lat; // Cartesian Y
        sumLon += state.lon; // Cartesian X
    }
    double centerLat = sumLat / state_history.size();
    double centerLon = sumLon / state_history.size();
    
    double dx = receivedLon - centerLon;
    double dy = receivedLat - centerLat;
    return std::sqrt(dx*dx + dy*dy);
}

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
    long sender;
    double pos_x;
    double pos_y;
    double speed;
    double heading;
    double accel;     // Computed during extraction
    double yawRate;   // Computed during extraction
    
    // For sorting
    bool operator<(const RawMessage& other) const {
        return rcvTime < other.rcvTime;
    }
};

struct PrevState {
    double time;
    double speed;
    double heading;
};

// -----------------------------------------------------------------------------
// Parquet Message Extractor
// -----------------------------------------------------------------------------
class ParquetMessageExtractor {
public:
    static std::vector<RawMessage> extractMessages(const std::string& dirPath) {
        std::vector<std::string> parquet_files;
        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (entry.path().extension() == ".parquet") {
                if (entry.path().filename().string().find("traceJSON") == 0) {
                    parquet_files.push_back(entry.path().string());
                }
            }
        }

        std::vector<RawMessage> all_messages;
        
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
            
            if (!type_col || !rcvTime_col || !sender_col || !pos_x_col || !pos_y_col || !spd_x_col || !spd_y_col || !hed_x_col || !hed_y_col) {
                continue;
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
                
                for (int64_t i = 0; i < type_arr->length(); ++i) {
                    if (!type_arr->IsNull(i) && type_arr->Value(i) == 3) {
                        RawMessage msg;
                        msg.rcvTime = time_arr->Value(i);
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
                        msg.accel = 0.0;
                        msg.yawRate = 0.0;
                        
                        all_messages.push_back(msg);
                    }
                }
            }
        }
        
        std::sort(all_messages.begin(), all_messages.end());
        computeDerivedKinematics(all_messages);
        
        return all_messages;
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
    
    std::map<long, std::deque<KinematicState>> m_cache;
    std::map<long, double> m_scores;

public:
    TrustEvaluationEngine(const std::string& drModelName, const std::string& trustModelName, 
                          double maxPosDev = 4.3506, double contextSpeedWeight = 5.2264, double contextYawWeight = 0.6536,
                          bool silentMode = false) {
        silent = silentMode;
        if (drModelName == "HistoryAwareCTRA") {
            drModel = std::make_unique<CartesianHistoryAwareCtraModel>();
        } else {
            drModel = std::make_unique<CartesianCtraDeadReckoningModel>();
        }
        
        trustEvaluator = TrustEvaluatorFactory::create(trustModelName);

        // values from offline training or overridden by CLI
        params.maxPosDev = maxPosDev;
        params.contextSpeedWeight = contextSpeedWeight;
        params.contextYawWeight = contextYawWeight;

        if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << "rcvTime,sender,score,distError,centroidDeviation\n";
#endif
        }
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
            std::deque<KinematicState> newHistory;
            newHistory.push_back(currentState);
            m_cache[msg.sender] = newHistory;
            if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.sender << ",1.0,0.0,0.0\n";
#endif
            }
            return;
        }

        auto& history = it->second;
        KinematicState pastState = history.back();
        
        double dt = currentState.time - pastState.time;
        if (dt <= 0.0) dt = 0.1;

        PredictedState expectedState;
        drModel->predict(history, pastState, expectedState, dt);

        double distError = drModel->computeError(expectedState, currentState);
        double centroidDeviation = calculateCentroidDeviation(history, currentState.lat, currentState.lon);

        TrustEvaluationContext context {
            history,
            pastState,
            currentState,
            m_scores[msg.sender],
            distError,
            centroidDeviation
        };

        double newScore = trustEvaluator->evaluateTrust(context, params);
        m_scores[msg.sender] = newScore;

        if (!silent) {
#ifndef DISABLE_OUTPUT
            std::cout << msg.rcvTime << "," << msg.sender << "," << newScore << "," 
                      << distError << "," << centroidDeviation << "\n";
#endif
        }

        history.push_back(currentState);
        if (history.size() > 5) {
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
    double maxPosDev = 4.3506;
    double speedWeight = 5.2264;
    double yawWeight = 0.6536;
    bool profileMode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dr" && i + 1 < argc) {
            drModelName = argv[++i];
        } else if (arg == "--trust" && i + 1 < argc) {
            trustModelName = argv[++i];
        } else if (arg == "--dir" && i + 1 < argc) {
            dirPath = argv[++i];
        } else if (arg == "--maxPosDev" && i + 1 < argc) {
            maxPosDev = std::stod(argv[++i]);
        } else if (arg == "--speedWeight" && i + 1 < argc) {
            speedWeight = std::stod(argv[++i]);
        } else if (arg == "--yawWeight" && i + 1 < argc) {
            yawWeight = std::stod(argv[++i]);
        } else if (arg == "--profile") {
            profileMode = true;
        }
    }
    
    if (dirPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " --dir <path> [--dr <model>] [--trust <model>]\n";
        return 1;
    }

    // 1. Extract and format messages
    auto messages = ParquetMessageExtractor::extractMessages(dirPath);
    
    // 2. Initialize engine
    TrustEvaluationEngine engine(drModelName, trustModelName, maxPosDev, speedWeight, yawWeight, profileMode);

    // 3. Process stream
    if (profileMode) {
        std::cout << "Starting performance profiling over " << messages.size() << " messages...\n";
        auto start = std::chrono::high_resolution_clock::now();
        
        for (const auto& msg : messages) {
            engine.processMessage(msg);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        
        std::cout << "----------------------------------------\n";
        std::cout << "PROFILING RESULTS\n";
        std::cout << "----------------------------------------\n";
        std::cout << "Total Execution Time: " << duration << " us\n";
        std::cout << "Messages Processed:   " << messages.size() << "\n";
        std::cout << "Avg Latency per Msg:  " << (double)duration / messages.size() << " us\n";
        std::cout << "----------------------------------------\n";
    } else {
        for (const auto& msg : messages) {
            engine.processMessage(msg);
        }
    }

    return 0;
}
