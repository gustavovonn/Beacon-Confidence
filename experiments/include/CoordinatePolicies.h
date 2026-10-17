#ifndef COORDINATE_POLICIES_H_
#define COORDINATE_POLICIES_H_

#include "IDeadReckoningModel.h"
#include "Constants.h"
#include <deque>
#include <cmath>
#include <algorithm>

// -----------------------------------------------------------------------------
// Cartesian Coordinate Policy (Euclidean 2D metric space, e.g. SUMO / VeReMi)
// -----------------------------------------------------------------------------
struct CartesianCoordinatePolicy {
    static constexpr const char* name() { return "Cartesian"; }

    /**
     * @brief CTRA kinematic integration in Cartesian space (X=East, Y=North).
     * In KinematicState / PredictedState: lon stores X [m], lat stores Y [m].
     */
    static void predictState(const KinematicState& lastState,
                             double a, double w, double dt,
                             PredictedState& nextState)
    {
        const double startX = lastState.lon;
        const double startY = lastState.lat;
        const double startSpeed = lastState.speed;
        const double startHeading = lastState.heading;
        const double headingRad = startHeading * ConstantsWGS84::DEG_TO_RAD;

        if (std::abs(w) < ConstantsApp::YAW_RATE_THRESHOLD)
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

            nextState.lat = startY + dy;
            nextState.lon = startX + dx;
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

        const double dx = inv_w * (-v_dt * cosH1 + startSpeed * cosH0) + a_over_w2 * (sinH1 - sinH0);
        const double dy = inv_w * ( v_dt * sinH1 - startSpeed * sinH0) + a_over_w2 * (cosH1 - cosH0);

        nextState.lat = startY + dy;
        nextState.lon = startX + dx;
        nextState.speed = std::max(0.0, v_dt);

        double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
        if (headingDeg < 0.0) headingDeg += 360.0;
        nextState.heading = headingDeg;
    }

    /**
     * @brief Euclidean distance between two points in meters.
     */
    static double distance(double pos1_a, double pos2_a,
                           double pos1_b, double pos2_b)
    {
        return std::hypot(pos1_b - pos1_a, pos2_b - pos2_a);
    }

    /**
     * @brief Inter-frame spatial displacement in meters.
     */
    static double displacement(double pos1_a, double pos2_a,
                               double pos1_b, double pos2_b)
    {
        return distance(pos1_a, pos2_a, pos1_b, pos2_b);
    }
};

// -----------------------------------------------------------------------------
// Geographic Coordinate Policy (WGS-84 Ellipsoid, e.g. GPS / Artery CAM)
// -----------------------------------------------------------------------------
struct Wgs84CoordinatePolicy {
    static constexpr const char* name() { return "WGS84"; }

    /**
     * @brief CTRA kinematic integration and ellipsoidal projection onto WGS-84.
     * In KinematicState / PredictedState: lat and lon are in degrees.
     */
    static void predictState(const KinematicState& lastState,
                             double a, double w, double dt,
                             PredictedState& nextState)
    {
        const double startLat = lastState.lat;
        const double startLon = lastState.lon;
        const double startSpeed = lastState.speed;
        const double startHeading = lastState.heading;

        const double latRad = startLat * ConstantsWGS84::DEG_TO_RAD;
        const double headingRad = startHeading * ConstantsWGS84::DEG_TO_RAD;

        const double R_phi = ConstantsWGS84::meridionalRadius(latRad);       // N-S [m]
        const double R_lambda = ConstantsWGS84::primeVerticalRadius(latRad); // E-W [m]
        const double cosLat = std::cos(latRad);

        if (std::abs(w) < ConstantsApp::YAW_RATE_THRESHOLD)
        {
            const double predictedSpeed = startSpeed + a * dt;
            double dist = startSpeed * dt + 0.5 * a * dt * dt;
            if (predictedSpeed <= 0.0 || dist < 0.0) {
                dist = 0.0;
                nextState.speed = 0.0;
            } else {
                nextState.speed = predictedSpeed;
            }

            const double dy = dist * std::cos(headingRad); // North [m]
            const double dx = dist * std::sin(headingRad); // East  [m]

            // to verify.
            // before:
            // nextState.lat = startLat + dy / R_phi;
            // nextState.lon = startLon + dx / (R_lambda * cosLat);
            nextState.lat = startLat + (dy / R_phi) * ConstantsWGS84::RAD_TO_DEG;
            nextState.lon = startLon + (dx / (R_lambda * cosLat)) * ConstantsWGS84::RAD_TO_DEG;
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

        // to verify.
        // before:
        // const double dx = inv_w     * ( v_dt * sinH1 - v * sinH0)
        //                 + a_over_w2 * (        cosH1     - cosH0);
        // const double dy = inv_w     * (-v_dt * cosH1 + v * cosH0)
        //                 + a_over_w2 * (        sinH1     - sinH0);

        const double dx = inv_w * (-v_dt * cosH1 + startSpeed * cosH0) + a_over_w2 * (sinH1 - sinH0); // East [m]
        const double dy = inv_w * ( v_dt * sinH1 - startSpeed * sinH0) + a_over_w2 * (cosH1 - cosH0); // North [m]

        // to verify.
        // before:
        // nextState.lat = startLat + dy / R_phi;
        // nextState.lon = startLon + dx / (R_lambda * cosLat);
        nextState.lat = startLat + (dy / R_phi) * ConstantsWGS84::RAD_TO_DEG;
        nextState.lon = startLon + (dx / (R_lambda * cosLat)) * ConstantsWGS84::RAD_TO_DEG;
        nextState.speed = std::max(0.0, v_dt);

        double headingDeg = std::fmod(newHeadingRad * ConstantsWGS84::RAD_TO_DEG, 360.0);
        if (headingDeg < 0.0) headingDeg += 360.0;
        nextState.heading = headingDeg;
    }

    /**
     * @brief Ellipsoidal equirectangular distance between two (lat, lon) points in meters.
     */
    static double distance(double lat1, double lon1,
                           double lat2, double lon2)
    {
        const double latRad1 = lat1 * ConstantsWGS84::DEG_TO_RAD;
        const double latRad2 = lat2 * ConstantsWGS84::DEG_TO_RAD;
        const double lonRad1 = lon1 * ConstantsWGS84::DEG_TO_RAD;
        const double lonRad2 = lon2 * ConstantsWGS84::DEG_TO_RAD;

        const double latMid = (latRad1 + latRad2) * 0.5;

        const double R_phi = ConstantsWGS84::meridionalRadius(latMid);
        const double R_lambda = ConstantsWGS84::primeVerticalRadius(latMid);

        const double dy = R_phi * (latRad2 - latRad1);
        const double dx = R_lambda * (lonRad2 - lonRad1) * std::cos(latMid);
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Inter-frame spatial displacement in meters.
     */
    static double displacement(double lat1, double lon1,
                               double lat2, double lon2)
    {
        return distance(lat1, lon1, lat2, lon2);
    }
};

#endif // COORDINATE_POLICIES_H_
