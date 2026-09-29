#ifndef CPM_TYPES_H_
#define CPM_TYPES_H_

#include "Confidence.h"
#include "Geodesy.h"
#include "IDeadReckoningModel.h"  // KinematicState
#include <cstdint>
#include <vector>

namespace artery {

/// Self claim carried by a CAM, already converted to the vehicle centre.
struct CamClaim {
    StationId author = 0;
    KinematicState state;  ///< centre position, heading clockwise from north, yaw rate clockwise-positive
    double length = 0.0;   ///< [m], 0 if unavailable
};

/// Object claim carried by a CPM (one perceived object).
struct ObjectClaim {
    StationId author = 0;
    uint16_t objectId = 0;  ///< author-local, anonymous
    double time = 0.0;      ///< measurement time [s]
    GeoPoint position;      ///< object centre
    double speed = 0.0;     ///< [m/s]
    double heading = 0.0;   ///< [deg clockwise from north]
    bool hasVelocity = false;
    double sensorVariance = 0.0;  ///< per-axis variance of the author's radar measurement [m^2] (set by the engine)
};

/// Parsed CPM.
struct CpmSnapshot {
    StationId author = 0;
    double time = 0.0;           ///< generation time [s]
    uint64_t referenceTaiMs = 0; ///< referenceTime as sent (evaluation bookkeeping only)
    GeoPoint referencePosition;  ///< author reference point as sent (front bumper for vehicles)
    double heading = 0.0;        ///< author heading [deg clockwise from north]
    std::vector<ObjectClaim> objects;
};

/// Something the receiver's own physical sensors currently detect.
struct EgoDetection {
    uint32_t id = 0;         ///< local tracking id
    GeoPoint position;       ///< object centre
    double speed = 0.0;      ///< [m/s]
    double heading = 0.0;    ///< [deg clockwise from north]
    bool hasVelocity = false;
};

/// The receiver's own pose and perception at one instant.
struct EgoView {
    StationId station = 0;
    double time = 0.0;
    GeoPoint centre;
    double heading = 0.0;
    double speed = 0.0;
    std::vector<EgoDetection> detections;
};

/**
 * Gating of two position estimates of one object. The position terms come from the error models
 * (ErrorModels.h): sigmaGPS is the VeReMi Extension per-axis GNSS error, sigmaHeading its heading error at
 * standstill, decaying with speed. Radar variances travel with each object (ObjectClaim::sensorVariance).
 */
struct AssociationParams {
    double kappaGate = 3.0;                 ///< gate multiplier on the composite 1-sigma position error
    double lambdaV = 1.0;                   ///< velocity weight in the assignment cost
    double sigmaGPS = 2.887;                ///< [m] per axis, 5 / sqrt(3)
    double sigmaHeading = 0.2015;           ///< [rad] at standstill, (20 / sqrt(3)) deg
    double headingSpeedDecay = 0.1;         ///< [s/m] heading error ~ exp(-decay * v)
    double sigmaVel = 2.0;                  ///< [m/s] speed tolerance of the assignment cost
    double aMax = 4.0;                      ///< [m/s^2] unmodelled acceleration over the projection horizon
};

/// A CAM track projected to the time of a CPM.
struct ProjectedTrack {
    StationId station = 0;
    KinematicState state;
    double syncDt = 0.0;  ///< extrapolation horizon used for the projection [s]
};

/// A resolved object claim: CPM object `objectIndex` is the vehicle `station`.
struct AssociationMatch {
    StationId station = 0;
    size_t objectIndex = 0;
};

}  // namespace artery

#endif  // CPM_TYPES_H_
