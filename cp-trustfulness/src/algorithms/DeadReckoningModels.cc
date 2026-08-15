#include "DeadReckoningModels.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace artery
{

static double computeGeoDistance(double lat1, double lon1, double lat2, double lon2)
{
    double latRad1 = lat1 * M_PI / 180.0;
    double latRad2 = lat2 * M_PI / 180.0;
    double lonRad1 = lon1 * M_PI / 180.0;
    double lonRad2 = lon2 * M_PI / 180.0;

    double x = (lonRad2 - lonRad1) * std::cos((latRad1 + latRad2) / 2.0);
    double y = (latRad2 - latRad1);
    return 6371000.0 * std::sqrt(x * x + y * y);
}

// -----------------------------------------------------------------------------
// Constant Velocity Model (CV)
// -----------------------------------------------------------------------------
void CvDeadReckoningModel::predict(VehicleState& state, double dt)
{
    // To be analysed
    throw std::runtime_error("To be analysed: CV model is to be implemented.");
}

double CvDeadReckoningModel::computeError(const VehicleState& state, double actualLat, double actualLon,
                                          double actualSpeed, double actualHeading)
{
    // To be analysed
    throw std::runtime_error("To be analysed: CV model is to be implemented.");
}

// -----------------------------------------------------------------------------
// Constant Acceleration Model (CA)
// -----------------------------------------------------------------------------
void CaDeadReckoningModel::predict(VehicleState& state, double dt)
{
    // To be analysed
    throw std::runtime_error("To be analysed: CA model is to be implemented.");
}

double CaDeadReckoningModel::computeError(const VehicleState& state, double actualLat, double actualLon,
                                          double actualSpeed, double actualHeading)
{
    // To be analysed
    throw std::runtime_error("To be analysed: CA model is to be implemented.");
}

// -----------------------------------------------------------------------------
// Constant Turn Rate & Acceleration Model (CTRA)
// -----------------------------------------------------------------------------
void CtraDeadReckoningModel::predict(VehicleState& state, double dt)
{
    if (dt <= 0.0) return;

    double headingRad = state.lastHeading * M_PI / 180.0;
    double yawRate = state.lastYawRate;

    if (std::abs(yawRate) < 1e-4) {
        // Fallback to CA model when driving nearly straight
        double dist = state.lastSpeed * dt + 0.5 * state.lastAccel * dt * dt;
        if (dist < 0.0) dist = 0.0;
        double dLat = (dist * std::cos(headingRad)) / 111320.0;
        double dLon = (dist * std::sin(headingRad)) / (111320.0 * std::cos(state.lastLat * M_PI / 180.0));
        state.expectedLat = state.lastLat + dLat;
        state.expectedLon = state.lastLon + dLon;
        state.expectedSpeed = std::max(0.0, state.lastSpeed + state.lastAccel * dt);
        state.expectedHeading = state.lastHeading;
    } else {
        double v = state.lastSpeed;
        double a = state.lastAccel;
        double w = yawRate;
        
        double v_dt = v + a * dt;
        double headingRad = state.lastHeading * M_PI / 180.0;
        double newHeadingRad = headingRad + w * dt;

        double inv_w = 1.0 / w;
        double a_over_w2 = a / (w * w);

        // Exact analytical integration of CTRA equations (SDAV formulation)
        // dx (East/Longitude displacement), dy (North/Latitude displacement)
        double dx = inv_w * (v_dt * std::sin(newHeadingRad) - v * std::sin(headingRad)) +
                    a_over_w2 * (std::cos(newHeadingRad) - std::cos(headingRad));

        double dy = inv_w * (-v_dt * std::cos(newHeadingRad) + v * std::cos(headingRad)) +
                    a_over_w2 * (std::sin(newHeadingRad) - std::sin(headingRad));

        double dLat = dy / 111320.0;
        double dLon = dx / (111320.0 * std::cos(state.lastLat * M_PI / 180.0));

        state.expectedLat = state.lastLat + dLat;
        state.expectedLon = state.lastLon + dLon;
        state.expectedSpeed = std::max(0.0, v_dt);
        state.expectedHeading = std::fmod(newHeadingRad * 180.0 / M_PI, 360.0);
    }
}

double CtraDeadReckoningModel::computeError(const VehicleState& state, double actualLat, double actualLon,
                                            double actualSpeed, double actualHeading)
{
    return computeGeoDistance(state.expectedLat, state.expectedLon, actualLat, actualLon);
}

// -----------------------------------------------------------------------------
// Extended Kalman Filter Model (EKF)
// -----------------------------------------------------------------------------
void EkfDeadReckoningModel::predict(VehicleState& state, double dt)
{
    // To be analysed
    throw std::runtime_error("To be analysed: EKF model is to be implemented.");
}

double EkfDeadReckoningModel::computeError(const VehicleState& state, double actualLat, double actualLon,
                                           double actualSpeed, double actualHeading)
{
    // To be analysed
    throw std::runtime_error("To be analysed: EKF model is to be implemented.");
}

// -----------------------------------------------------------------------------
// History-Aware CTRA Model (Windowed CTRA with Least Squares)
// -----------------------------------------------------------------------------

double HistoryAwareCtraModel::unwrapHeading(double currentHeading, double previousHeading)
{
    double diff = currentHeading - previousHeading;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return previousHeading + diff;
}

void HistoryAwareCtraModel::predict(VehicleState& state, double dt)
{
    if (dt <= 0.0) return;

    if (state.history.size() < 3) {
        // Fallback to standard CTRA logic if not enough history
        CtraDeadReckoningModel fallback;
        fallback.predict(state, dt);
        return;
    }

    double sum_t = 0.0, sum_v = 0.0, sum_h = 0.0;
    double sum_t2 = 0.0, sum_tv = 0.0, sum_th = 0.0;
    
    double t0 = state.history.front().time.dbl();
    double prev_heading = state.history.front().heading;
    
    size_t n = state.history.size();
    
    for (size_t i = 0; i < n; ++i) {
        double t = state.history[i].time.dbl() - t0;
        double v = state.history[i].speed;
        double h = state.history[i].heading;
        
        if (i > 0) {
            h = unwrapHeading(h, prev_heading);
            prev_heading = h;
        }
        
        sum_t += t;
        sum_v += v;
        sum_h += h;
        sum_t2 += t * t;
        sum_tv += t * v;
        sum_th += t * h;
    }
    
    // Calculate slopes (least squares)
    double denominator = n * sum_t2 - sum_t * sum_t;
    double a = state.lastAccel;
    double w = state.lastYawRate;
    
    if (std::abs(denominator) > 1e-6) {
        a = (n * sum_tv - sum_t * sum_v) / denominator;
        
        double w_deg = (n * sum_th - sum_t * sum_h) / denominator; // deg/s
        w = w_deg * M_PI / 180.0; // convert to rad/s
    }

    // Now use CTRA math with smoothed a and w, starting from last known state
    double headingRad = state.lastHeading * M_PI / 180.0;
    
    if (std::abs(w) < 1e-4) {
        // Fallback to CA model when driving nearly straight
        double dist = state.lastSpeed * dt + 0.5 * a * dt * dt;
        if (dist < 0.0) dist = 0.0;
        double dLat = (dist * std::cos(headingRad)) / 111320.0;
        double dLon = (dist * std::sin(headingRad)) / (111320.0 * std::cos(state.lastLat * M_PI / 180.0));
        state.expectedLat = state.lastLat + dLat;
        state.expectedLon = state.lastLon + dLon;
        state.expectedSpeed = std::max(0.0, state.lastSpeed + a * dt);
        state.expectedHeading = state.lastHeading;
    } else {
        double v = state.lastSpeed;
        double v_dt = v + a * dt;
        double newHeadingRad = headingRad + w * dt;

        double inv_w = 1.0 / w;
        double a_over_w2 = a / (w * w);

        double dx = inv_w * (v_dt * std::sin(newHeadingRad) - v * std::sin(headingRad)) +
                    a_over_w2 * (std::cos(newHeadingRad) - std::cos(headingRad));

        double dy = inv_w * (-v_dt * std::cos(newHeadingRad) + v * std::cos(headingRad)) +
                    a_over_w2 * (std::sin(newHeadingRad) - std::sin(headingRad));

        double dLat = dy / 111320.0;
        double dLon = dx / (111320.0 * std::cos(state.lastLat * M_PI / 180.0));

        state.expectedLat = state.lastLat + dLat;
        state.expectedLon = state.lastLon + dLon;
        state.expectedSpeed = std::max(0.0, v_dt);
        
        double finalHeading = std::fmod(newHeadingRad * 180.0 / M_PI, 360.0);
        if (finalHeading < 0) finalHeading += 360.0;
        state.expectedHeading = finalHeading;
    }
}

double HistoryAwareCtraModel::computeError(const VehicleState& state, double actualLat, double actualLon,
                                            double actualSpeed, double actualHeading)
{
    return computeGeoDistance(state.expectedLat, state.expectedLon, actualLat, actualLon);
}

// -----------------------------------------------------------------------------
// Factory Pattern Implementation
// -----------------------------------------------------------------------------
std::unique_ptr<IDeadReckoningModel> DeadReckoningModelFactory::create(const std::string& name)
{
    if (name == "CA" || name == "ca") {
        return std::make_unique<CaDeadReckoningModel>();
    } else if (name == "CV" || name == "cv") {
        return std::make_unique<CvDeadReckoningModel>();
    } else if (name == "HistoryAwareCTRA" || name == "historyawarectra") {
        return std::make_unique<HistoryAwareCtraModel>();
    } else if (name == "EKF" || name == "ekf") {
        return std::make_unique<EkfDeadReckoningModel>();
    }
    // Default to CTRA model (validated)
    return std::make_unique<CtraDeadReckoningModel>();
}

} // namespace artery
