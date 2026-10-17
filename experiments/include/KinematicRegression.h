#ifndef KINEMATIC_REGRESSION_H_
#define KINEMATIC_REGRESSION_H_

#include "IDeadReckoningModel.h"
#include "Constants.h"
#include <deque>
#include <cmath>

namespace KinematicRegression {

/**
 * @brief Unwraps heading angles to eliminate ±180° / 0°-360° discontinuities
 * during numerical differentiation and linear regression.
 */
inline double unwrapHeading(double current, double previous)
{
    // Precondition: currentHeading in [0, 360)
    double diff = current - previous;
    // Reducing diff to (-180, 180]
    diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
    return previous + diff;
}

/**
 * @brief Ordinary Least Squares (OLS) linear regression over sliding window
 * of KinematicStates to estimate smoothed acceleration (â) and yaw rate (ω̂).
 */
inline void fitOLS(const std::deque<KinematicState>& history,
                   double fallbackAccel, double fallbackYawRate,
                   double& a_hat, double& w_hat)
{
    if (history.size() < 3)
    {
        a_hat = fallbackAccel;
        w_hat = fallbackYawRate;
        return;
    }

    const size_t N = history.size();
    const double t0 = history.front().time;

    double sum_tau = 0.0;
    double sum_tau2 = 0.0;
    double sum_v = 0.0;
    double sum_tau_v = 0.0;
    double sum_theta = 0.0;
    double sum_tau_theta = 0.0;

    double prev_unwrapped = history.front().heading;

    for (size_t i = 0; i < N; ++i)
    {
        const double tau = history[i].time - t0;
        const double v = history[i].speed;

        double theta = history[i].heading;
        if (i > 0)
        {
            theta = unwrapHeading(theta, prev_unwrapped);
        }
        prev_unwrapped = theta;

        sum_tau += tau;
        sum_tau2 += tau * tau;
        sum_v += v;
        sum_tau_v += tau * v;
        sum_theta += theta;
        sum_tau_theta += tau * theta;
    }

    const double denom = static_cast<double>(N) * sum_tau2 - sum_tau * sum_tau;

    if (std::abs(denom) < ConstantsApp::OLS_DENOM_THRESHOLD)
    {
        // Degenerate fit (all timestamps nearly identical): keep last-known values.
        a_hat = fallbackAccel;
        w_hat = fallbackYawRate;
        return;
    }

    const double inv_denom = 1.0 / denom;
    a_hat = (static_cast<double>(N) * sum_tau_v - sum_tau * sum_v) * inv_denom;

    const double slope_deg = (static_cast<double>(N) * sum_tau_theta - sum_tau * sum_theta) * inv_denom;
    w_hat = slope_deg * ConstantsWGS84::DEG_TO_RAD;
}

} // namespace KinematicRegression

#endif // KINEMATIC_REGRESSION_H_
