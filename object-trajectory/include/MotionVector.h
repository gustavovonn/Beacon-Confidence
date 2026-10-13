#ifndef OT_MOTION_VECTOR_H_
#define OT_MOTION_VECTOR_H_

/**
 * The one unit of content. A CAM and a CPM object both become this, so past the parser the
 * container stops mattering.
 *
 * Field set follows the SmartData ecosystem's motion vector
 * (sdav/smartdata/include/transformers/av/dynamics_state_kf_on_hardware.h, `_Motion_Vector`, and
 * sdav/external/AV_Modules_SD/obj_rec_track_lib/motion_vectors.py). Two of its fields exist here for
 * reasons the previous paradigm did not have:
 *   - `objectClass` and `dimension` help the assignment: two crossing objects of different class or
 *     size cannot be confused, which attacks the track-swap problem at its root.
 *   - `uncertainty` is where the ecosystem already expects a confidence to live, so what this engine
 *     publishes is consumable without inventing a field.
 *
 * Positions are metres in a local tangent plane fixed by the dataset's manifest, NOT degrees. The
 * generator does the geodetic conversion once; nothing downstream touches WGS84.
 */

#include <cstdint>

namespace ot {

/// Where the vector came from. Kept for provenance, never for judging content.
enum class Origin : uint8_t { Cam = 0, CpmObject = 1 };

struct MotionVector {
    double t = 0.0;          ///< [s] when the content refers to, not when it arrived
    double x = 0.0;          ///< [m] east in the local plane
    double y = 0.0;          ///< [m] north
    double speed = 0.0;      ///< [m/s]
    double heading = 0.0;    ///< [rad] clockwise from north
    double yawRate = 0.0;    ///< [rad/s], 0 when the wire carries none
    double accel = 0.0;      ///< [m/s^2], 0 when the wire carries none

    double length = 0.0;     ///< [m] bounding box, 0 when unknown
    double width = 0.0;      ///< [m]
    int objectClass = 0;     ///< 0 = unknown

    Origin origin = Origin::Cam;
    uint64_t author = 0;     ///< station that SENT this vector (the CAM sender or the CPM author)
    uint32_t objectId = 0;   ///< author-local id for a perceived object; unused for a CAM

    /// Evaluation only. The station this vector is really about, from the generator's oracle.
    /// Never read by the engine; the experiment runner uses it to score association.
    int64_t trueStation = -1;

    bool hasYawRate = false;
    bool hasVelocity = true;
};

}  // namespace ot

#endif  // OT_MOTION_VECTOR_H_
