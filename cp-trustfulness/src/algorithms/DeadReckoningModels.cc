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

        const double R_phi = ConstantsWGS84::meridionalRadius(latMid);
        const double R_lambda = ConstantsWGS84::primeVerticalRadius(latMid);

        const double dy = R_phi    * (latRad2 - latRad1);
        const double dx = R_lambda * (lonRad2 - lonRad1) * std::cos(latMid);
        return std::sqrt(dx * dx + dy * dy);
    }

    static void ctraIntegrateAndProject(VehicleState& state, double dt, double a, double w)
    {
        const double latRad = state.lastLat * ConstantsWGS84::DEG_TO_RAD;
        const double headingRad = state.lastHeading * ConstantsWGS84::DEG_TO_RAD;

        const double R_phi = ConstantsWGS84::meridionalRadius(latRad);    // N-S [m]
        const double R_lambda = ConstantsWGS84::primeVerticalRadius(latRad); // E-W [m]
        const double cosLat = std::cos(latRad);

        // straight-line integration
        if (std::abs(w) < ConstantsApp::YAW_RATE_THRESHOLD)
        {
            const double v = state.lastSpeed;
            const double predictedSpeed = v + a * dt;

            double dist;
            if (predictedSpeed <= 0.0 || dist < 0.0)
            {
                dist = 0.0;
                state.expectedSpeed = 0.0;
            }
            else
            {
                dist = v * dt + 0.5 * a * dt * dt;
                state.expectedSpeed = predictedSpeed;
            }

            const double dy = dist * std::cos(headingRad); // North [m]
            const double dx = dist * std::sin(headingRad); // East  [m]

            state.expectedLat = state.lastLat + dy / R_phi;
            state.expectedLon = state.lastLon + dx / (R_lambda * cosLat);
            state.expectedHeading = state.lastHeading; // unchanged in straight-line
            return;
        }

        // curved path
        const double v = state.lastSpeed;
        const double v_dt = v + a * dt;           // v' = speed at end of interval
        const double newHeadingRad = headingRad + w * dt;  // θ' = heading at end of interval

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

        state.expectedLat = state.lastLat + dy / R_phi;
        state.expectedLon = state.lastLon + dx / (R_lambda * cosLat);
        state.expectedSpeed = std::max(0.0, v_dt);

        // Normalise heading to [0°, 360°).
        double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
        if (headingDeg < 0.0) headingDeg += 360.0;
        state.expectedHeading = headingDeg;
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
        ctraIntegrateAndProject(state, dt, state.lastAccel, state.lastYawRate);
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

        // Not enough history for a meaningful OLS fit: delegate to single-step CTRA.
        if (state.history.size() < 3)
        {
            CtraDeadReckoningModel fallback;
            fallback.predict(state, dt);
            return;
        }

        // OLS temporal regression over the history window
        double sum_t  = 0.0, sum_v  = 0.0, sum_h  = 0.0;
        double sum_t2 = 0.0, sum_tv = 0.0, sum_th = 0.0;

        const double t0 = state.history.front().time.dbl();
        double prevHeading = state.history.front().heading;
        const size_t n = state.history.size();

        for (size_t i = 0; i < n; ++i)
        {
            const double t = state.history[i].time.dbl() - t0;
            const double v = state.history[i].speed;
            double h = state.history[i].heading;

            if (i > 0)
            {
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

        // OLS denominator
        const double denom = static_cast<double>(n) * sum_t2 - sum_t * sum_t;

        double a, w;
        if (std::abs(denom) < ConstantsApp::OLS_DENOM_THRESHOLD)
        {
            // Degenerate fit (all timestamps nearly identical): keep last-known values.
            a = state.lastAccel;
            w = state.lastYawRate;
        }
        else
        {
            a = (static_cast<double>(n) * sum_tv - sum_t * sum_v) / denom;
            const double w_deg = (static_cast<double>(n) * sum_th - sum_t * sum_h) / denom;
            w = w_deg * ConstantsWGS84::DEG_TO_RAD;
        }

        ctraIntegrateAndProject(state, dt, a, w);
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