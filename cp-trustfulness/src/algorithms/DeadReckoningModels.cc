#include "DeadReckoningModels.h"
#include "Constants.h"
#include <cmath>
#include <stdexcept>
#include <deque>

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

    static void ctraIntegrateAndProject(double startLat, double startLon, double startSpeed, double startHeading, 
                                        double a, double w, double dt, PredictedState& nextState)
    {
        const double latRad = startLat * ConstantsWGS84::DEG_TO_RAD;
        const double headingRad = startHeading * ConstantsWGS84::DEG_TO_RAD;

        const double R_phi = ConstantsWGS84::meridionalRadius(latRad);    // N-S [m]
        const double R_lambda = ConstantsWGS84::primeVerticalRadius(latRad); // E-W [m]
        const double cosLat = std::cos(latRad);

        // straight-line integration
        if (std::abs(w) < ConstantsApp::YAW_RATE_THRESHOLD)
        {
            const double v = startSpeed;
            const double predictedSpeed = v + a * dt;

            double dist = v * dt + 0.5 * a * dt * dt;
            if (predictedSpeed <= 0.0 || dist < 0.0)
            {
                dist = 0.0;
                nextState.speed = 0.0;
            }
            else
            {
                nextState.speed = predictedSpeed;
            }

            const double dy = dist * std::cos(headingRad); // North [m]
            const double dx = dist * std::sin(headingRad); // East  [m]

            nextState.lat = startLat + dy / R_phi;
            nextState.lon = startLon + dx / (R_lambda * cosLat);
            nextState.heading = startHeading; // unchanged in straight-line
            return;
        }

        // curved path
        const double v = startSpeed;
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

        nextState.lat = startLat + dy / R_phi;
        nextState.lon = startLon + dx / (R_lambda * cosLat);
        nextState.speed = std::max(0.0, v_dt);

        // Normalise heading to [0°, 360°).
        double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
        if (headingDeg < 0.0) headingDeg += 360.0;
        nextState.heading = headingDeg;
    }

    // -----------------------------------------------------------------------------
    // Constant Turn Rate & Acceleration Model (CTRA)
    // -----------------------------------------------------------------------------
    void CtraDeadReckoningModel::predict(const std::deque<KinematicState>& history, 
                                         const KinematicState& lastState, 
                                         PredictedState& nextState, 
                                         double dt)
    {
        if (dt <= 0.0) return;
        ctraIntegrateAndProject(lastState.lat, lastState.lon, lastState.speed, lastState.heading, 
                                lastState.accel, lastState.yawRate, dt, nextState);
    }

    double CtraDeadReckoningModel::computeError(const PredictedState& expected, 
                                                const KinematicState& actual)
    {
        return computeGeoDistance(expected.lat, expected.lon, actual.lat, actual.lon);
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

    void HistoryAwareCtraModel::predict(const std::deque<KinematicState>& history, 
                                        const KinematicState& lastState, 
                                        PredictedState& nextState, 
                                        double dt)
    {
        if (dt <= 0.0) return;

        // Not enough history for a meaningful OLS fit: delegate to single-step CTRA.
        if (history.size() < 3)
        {
            CtraDeadReckoningModel fallback;
            fallback.predict(history, lastState, nextState, dt);
            return;
        }

        // OLS temporal regression over the history window
        double sum_t  = 0.0, sum_v  = 0.0, sum_h  = 0.0;
        double sum_t2 = 0.0, sum_tv = 0.0, sum_th = 0.0;

        const double t0 = history.front().time;
        double prevHeading = history.front().heading;
        const size_t n = history.size();

        for (size_t i = 0; i < n; ++i)
        {
            const double t = history[i].time - t0;
            const double v = history[i].speed;
            double h = history[i].heading;

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
            a = lastState.accel;
            w = lastState.yawRate;
        }
        else
        {
            a = (static_cast<double>(n) * sum_tv - sum_t * sum_v) / denom;
            const double w_deg = (static_cast<double>(n) * sum_th - sum_t * sum_h) / denom;
            w = w_deg * ConstantsWGS84::DEG_TO_RAD;
        }

        ctraIntegrateAndProject(lastState.lat, lastState.lon, lastState.speed, lastState.heading, 
                                a, w, dt, nextState);
    }

    double HistoryAwareCtraModel::computeError(const PredictedState& expected, 
                                               const KinematicState& actual)
    {
        return computeGeoDistance(expected.lat, expected.lon, actual.lat, actual.lon);
    }

    // -----------------------------------------------------------------------------
    // Factory Pattern Implementation
    // -----------------------------------------------------------------------------
    std::unique_ptr<IDeadReckoningModel> DeadReckoningModelFactory::create(const std::string& name)
    {
        if (name == "HistoryAwareCTRA" || name == "historyawarectra") {
            return std::make_unique<HistoryAwareCtraModel>();
        }

        // Default to CTRA model
        return std::make_unique<CtraDeadReckoningModel>();
    }

} // namespace artery