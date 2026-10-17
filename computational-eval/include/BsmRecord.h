#ifndef BSM_RECORD_H_
#define BSM_RECORD_H_

#include <cstdint>

struct BsmRecord {
    double  arrivalTime     = 0.0;
    int64_t sender          = -1;
    int64_t receiverId      = -1;
    int64_t receiverPseudo  = -1;

    double px  = 0.0, py  = 0.0;   // Reported position.
    double pnx = 0.0, pny = 0.0;   // Position confidence / noise.
    double sx  = 0.0, sy  = 0.0;   // Reported velocity.
    double snx = 0.0, sny = 0.0;   // Velocity confidence / noise.
    double ax  = 0.0, ay  = 0.0;   // Reported acceleration.
    double anx = 0.0, any = 0.0;   // Acceleration confidence / noise.
    double hx  = 1.0, hy  = 0.0;   // Reported heading vector.
    double hnx = 0.0, hny = 0.0;   // Heading confidence / noise.

    double epx = 0.0, epy = 0.0;   // Ego (receiver) position.
    double ehx = 1.0, ehy = 0.0;   // Ego heading.
    double esx = 0.0, esy = 0.0;   // Ego velocity.

    bool operator<(const BsmRecord& other) const {
        return arrivalTime < other.arrivalTime;
    }
};

#endif // BSM_RECORD_H_
