#pragma once

#include "BsmRecord.h"
#include "BasicSafetyMessage.h"
#include "ReceivedVector.h"
#include "MotionVectorAdapter.h"
#include <cmath>
#include <vector>

namespace BsmAdapter {

inline std::vector<BasicSafetyMessage> toBasicSafetyMessages(const std::vector<BsmRecord>& records)
{
    std::vector<BasicSafetyMessage> out;
    out.reserve(records.size());

    for (const auto& rec : records) {
        BasicSafetyMessage msg;
        msg.arrivalTime     = rec.arrivalTime;
        msg.pseudonym       = rec.sender;
        msg.receiver_id     = rec.receiverId;
        msg.receiver_pseudo = rec.receiverPseudo;

        msg.pos       = veins::Coord(rec.px, rec.py, 0);
        msg.posConf   = veins::Coord(rec.pnx, rec.pny, 0);

        msg.speed     = veins::Coord(rec.sx, rec.sy, 0);
        msg.speedConf = veins::Coord(rec.snx, rec.sny, 0);

        msg.accel     = veins::Coord(rec.ax, rec.ay, 0);
        msg.accelConf = veins::Coord(rec.anx, rec.any, 0);

        msg.heading     = veins::Coord(rec.hx, rec.hy, 0);
        msg.headingConf = veins::Coord(rec.hnx, rec.hny, 0);

        msg.recv_pos     = veins::Coord(rec.epx, rec.epy, 0);
        msg.recv_heading = veins::Coord(rec.ehx, rec.ehy, 0);
        msg.recv_speed   = veins::Coord(rec.esx, rec.esy, 0);

        out.push_back(msg);
    }
    return out;
}

/// Map onto the confidence engine's motion-vector model.
inline std::vector<ReceivedVector> toMotionVectors(const std::vector<BsmRecord>& records)
{
    std::vector<ReceivedVector> out;
    out.reserve(records.size());

    for (const auto& rec : records) {
        ReceivedVector rv;
        rv.receiver = static_cast<long>(rec.receiverId);
        rv.sender   = static_cast<long>(rec.sender);

        rv.mv.time    = rec.arrivalTime;
        rv.mv.lon     = rec.px;
        rv.mv.lat     = rec.py;
        rv.mv.speed   = std::sqrt(rec.sx * rec.sx + rec.sy * rec.sy);
        rv.mv.heading = MotionVectorAdapter::headingDegrees(rec.hx, rec.hy);

        // REVIEW: rec.ax / rec.ay carry the reported acceleration but are dropped;
        // the derivative is reconstructed instead (see applyFiniteDifference, and
        // the engine's OLS regression which supersedes both).
        rv.mv.accel   = 0.0;
        rv.mv.yawRate = 0.0;

        out.push_back(rv);
    }
    return out;
}

} // namespace BsmAdapter
