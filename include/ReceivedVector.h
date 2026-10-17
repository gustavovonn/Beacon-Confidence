#ifndef RECEIVED_VECTOR_H_
#define RECEIVED_VECTOR_H_

#include "MotionTypes.h"

/**
 * @brief One received kinematic beacon.
 *
 * Carries the motion vector (phi, lambda, theta, v, a, omega) together with the
 * identities needed to route it to a tracking history and to label output rows.
 * The engine itself only consumes (senderId, KinematicState); this struct is the
 * loader-side envelope shared by the experiment front-ends.
 */
struct ReceivedVector {
    long receiver = 0;   // Ego/receiving node; 0 when the trace does not record one.
    long sender   = 0;   // Sender identifier (CAM stationId in a real deployment).
    KinematicState mv;   // The reported motion vector.

    bool operator<(const ReceivedVector& other) const {
        return mv.time < other.mv.time;
    }
};

#endif // RECEIVED_VECTOR_H_
