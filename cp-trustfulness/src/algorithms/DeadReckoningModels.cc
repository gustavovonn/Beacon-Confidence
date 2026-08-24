#include "DeadReckoningModels.h"
#include "Constants.h"
#include <cmath>
#include <stdexcept>

namespace artery
{

static double computeGeoDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double latRad1 = lat1 * ConstantsWGS84::DEG_TO_RAD;
    const double latRad2 = lat2 * ConstantsWGS84::DEG_TO_RAD;
    const double lonRad1 = lon1 * ConstantsWGS84::DEG_TO_RAD;
    const double lonRad2 = lon2 * ConstantsWGS84::DEG_TO_RAD;

    const double latMid = (latRad1 + latRad2) / 2.0;

    // Use WGS84 radii at the midpoint latitude, consistent with the
    // CTRA predictor, instead of a fixed spherical mean radius.
    const double R_phi = ConstantsWGS84::meridionalRadius(latMid);
    const double R_lambda = ConstantsWGS84::primeVerticalRadius(latMid);

    const double dy = R_phi    * (latRad2 - latRad1);
    const double dx = R_lambda * (lonRad2 - lonRad1) * std::cos(latMid);
    return std::sqrt(dx * dx + dy * dy);
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
 
    const double latRad = state.lastLat * ConstantsWGS84::DEG_TO_RAD;
    const double headingRad = state.lastHeading * ConstantsWGS84::DEG_TO_RAD;
    const double yawRate = state.lastYawRate;
 
    const double R_phi    = ConstantsWGS84::meridionalRadius(latRad);   // N-S
    const double R_lambda = ConstantsWGS84::primeVerticalRadius(latRad); // E-W
    const double cosLat   = std::cos(latRad);
 
    // -------------------------------------------------------------------------
    // Nearly straight: fall back to CA model.
    // -------------------------------------------------------------------------
    if (std::abs(yawRate) < ConstantsApp::YAW_RATE_THRESHOLD)
    {
        double dist = state.lastSpeed * dt + 0.5 * state.lastAccel * dt * dt;

        // Clamp to zero: the vehicle cannot reverse under this model.
        // expectedSpeed is computed from the same kinematic limit so the two
        // stay consistent — if dist is clamped, speed must also be zero.
        const double predictedSpeed = state.lastSpeed + state.lastAccel * dt;
        if (predictedSpeed <= 0.0 || dist < 0.0)
        {
            dist = 0.0;
            state.expectedSpeed = 0.0;
        }
        else
        {
            state.expectedSpeed = predictedSpeed;
        }
 
        // North (dy) and East (dx) metric displacements
        const double dy = dist * std::cos(headingRad);
        const double dx = dist * std::sin(headingRad);
 
        state.expectedLat = state.lastLat + dy / R_phi;
        state.expectedLon = state.lastLon + dx / (R_lambda * cosLat);
        // Heading is unchanged in a straight-line (zero yaw-rate) prediction.
        state.expectedHeading = state.lastHeading;
        return;
    }
 
    // -------------------------------------------------------------------------
    // Curved path: CTRA integration
    // -------------------------------------------------------------------------
    const double v = state.lastSpeed;
    const double a = state.lastAccel;
    const double w = yawRate;
    const double v_dt = v + a * dt;  // speed at end of interval
    const double newHeadingRad = headingRad + w * dt;
 
    const double inv_w = 1.0 / w;
    const double a_over_w2 = a / (w * w);
 
    const double sinH0 = std::sin(headingRad);
    const double cosH0 = std::cos(headingRad);
    const double sinH1 = std::sin(newHeadingRad);
    const double cosH1 = std::cos(newHeadingRad);
 
    const double dx = inv_w     * ( v_dt * sinH1 - v * sinH0)
                    + a_over_w2 * (        cosH1     - cosH0);
 
    const double dy = inv_w     * (-v_dt * cosH1 + v * cosH0)
                    + a_over_w2 * (        sinH1     - sinH0);
 
    // Project metric displacements to geographic coordinates (WGS84)
    state.expectedLat = state.lastLat + dy / R_phi;
    state.expectedLon = state.lastLon + dx / (R_lambda * cosLat);
 
    // Speed: clamp to zero (no reversing)
    state.expectedSpeed = std::max(0.0, v_dt);
 
    // Heading: convert back to degrees and normalise to [0°, 360°).
    double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
    if (headingDeg < 0.0) headingDeg += 360.0;
    state.expectedHeading = headingDeg;
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
    // Precondition: currentHeading in [0, 360)
    double diff = currentHeading - previousHeading;
    // Reducing diff to (-180, 180]
    diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
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
    
    for (size_t i = 0; i < n; ++i) 
    {
        double t = state.history[i].time.dbl() - t0;
        double v = state.history[i].speed;
        double h = state.history[i].heading;
        
        if (i > 0) 
        {
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

    double a = (n * sum_tv - sum_t * sum_v) / denominator;
    double w_deg = (static_cast<double>(n) * sum_th - sum_t * sum_h) / denominator;
    double w = w_deg * ConstantsWGS84::DEG_TO_RAD;
    
    if (std::abs(denominator) < ConstantsApp::OLS_DENOM_THRESHOLD) {
        a = state.lastAccel;
        w = state.lastYawRate;
    }

    // Now use CTRA math with smoothed a and w, starting from last known state
    double headingRad = state.lastHeading * ConstantsWGS84::DEG_TO_RAD;

    if (std::abs(w) < ConstantsApp::YAW_RATE_THRESHOLD) {
        // Fallback to CA model when driving nearly straight
        double dist = state.lastSpeed * dt + 0.5 * a * dt * dt;
        if (dist < 0.0) dist = 0.0;
        double dLat = (dist * std::cos(headingRad)) / ConstantsFixedLatLong::R_PHI_DEG;
        double dLon = (dist * std::sin(headingRad)) / (ConstantsFixedLatLong::R_LAMBDA_DEG * std::cos(state.lastLat * ConstantsWGS84::DEG_TO_RAD));
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

        double dLat = dy / ConstantsFixedLatLong::R_PHI_DEG;
        double dLon = dx / (ConstantsFixedLatLong::R_LAMBDA_DEG * std::cos(state.lastLat * ConstantsWGS84::DEG_TO_RAD));

        state.expectedLat = state.lastLat + dLat;
        state.expectedLon = state.lastLon + dLon;
        state.expectedSpeed = std::max(0.0, v_dt);
 
        // Normalise heading to [0°, 360°) — std::fmod alone returns negative
        // values for left-turn headings; explicit wrap required.
        double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
        if (headingDeg < 0.0) headingDeg += 360.0;
        state.expectedHeading = headingDeg;

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
