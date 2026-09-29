#ifndef TEMPORAL_TRACK_SYNCHRONIZER_H_
#define TEMPORAL_TRACK_SYNCHRONIZER_H_

#include "CoordinatePolicies.h"
#include "CpmTypes.h"
#include "IDeadReckoningModel.h"
#include <deque>
#include <map>
#include <set>
#include <vector>

namespace artery {

enum class SyncMode { None = 0, ConstantVelocity = 1, Ctra = 2 };

class TemporalTrackSynchronizer {
public:
    /**
     * @brief Projects cached CAM tracks to `targetTime` (the time of a CPM).
     *
     * Tracks whose last state is more than 50 ms in the future or older than `maxSyncDt`
     * are dropped. The extrapolation horizon is kept in ProjectedTrack::syncDt so the
     * association gate can widen with it.
     */
    static std::vector<ProjectedTrack> synchronize(const std::map<StationId, std::deque<KinematicState>>& cache,
                                                   double targetTime, SyncMode mode, double maxSyncDt,
                                                   IDeadReckoningModel* drModel,
                                                   const std::set<StationId>& excluded = {})
    {
        std::vector<ProjectedTrack> projected;
        for (const auto& entry : cache) {
            const std::deque<KinematicState>& history = entry.second;
            if (history.empty() || excluded.count(entry.first)) {
                continue;
            }
            const KinematicState& last = history.back();
            double dt = targetTime - last.time;
            if (dt < -0.05 || dt > maxSyncDt) {
                continue;
            }
            dt = std::max(0.0, dt);

            ProjectedTrack track;
            track.station = entry.first;
            track.state = project(history, dt, mode, drModel);
            track.state.time = targetTime;
            track.syncDt = dt;
            projected.push_back(track);
        }
        return projected;
    }

    static KinematicState project(const std::deque<KinematicState>& history, double dt, SyncMode mode,
                                  IDeadReckoningModel* drModel)
    {
        const KinematicState& last = history.back();
        KinematicState s = last;
        if (mode == SyncMode::None || dt <= 1e-4) {
            return s;
        }
        PredictedState ps;
        if (mode == SyncMode::ConstantVelocity) {
            Wgs84CoordinatePolicy::predictState(last, 0.0, 0.0, dt, ps);
        } else if (drModel != nullptr) {
            drModel->predict(history, last, ps, dt);
        } else {
            Wgs84CoordinatePolicy::predictState(last, last.accel, last.yawRate, dt, ps);
        }
        s.lat = ps.lat;
        s.lon = ps.lon;
        s.speed = ps.speed;
        s.heading = ps.heading;
        return s;
    }
};

}  // namespace artery

#endif  // TEMPORAL_TRACK_SYNCHRONIZER_H_
