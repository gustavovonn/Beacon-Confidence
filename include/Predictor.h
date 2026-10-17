#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include "EngineConfig.h"
#include "MotionModels.h"
#include "MotionModels.h"
#include <deque>
#include <map>
#include <memory>
#include <string>

/**
 * @brief History-aware kinematic predictor (paper Sec. IV-A).
 *
 * Owns the bounded per-sender tracking history (the sliding window of the W most
 * recent snapshots) and the motion model used to project the expected next state.
 * OLS temporal regression over the window is performed inside the history-aware
 * motion model; see KinematicRegression::fitOLS.
 */
class Predictor {
public:
    Predictor(const std::string& motionModelName,
              const EngineConfig& config,
              const std::string& coordSystem = "cartesian")
        : m_cfg(config)
    {
        m_model = MotionModelFactory::create(motionModelName, coordSystem);
    }

    /// True once at least one motion vector has been recorded for this sender.
    bool isTracked(long senderId) const
    {
        return m_history.find(senderId) != m_history.end();
    }

    // Tracking history for a sender.
    const std::deque<KinematicState>& history(long senderId) const
    {
        static const std::deque<KinematicState> kNoHistory;
        auto it = m_history.find(senderId);
        return (it != m_history.end()) ? it->second : kNoHistory;
    }

    // Register a sender in the history cache, starting from this motion vector.
    void cacheRegister(long senderId, const KinematicState& state)
    {
        std::deque<KinematicState> fresh;
        fresh.push_back(state);
        m_history[senderId] = std::move(fresh);
    }

    /// Discard stale context and re-anchor tracking on this motion vector
    void purge(long senderId, const KinematicState& state)
    {
        auto& h = m_history[senderId];
        h.clear();
        h.push_back(state);
    }

    /// Append a verified motion vector, trimming the window to W snapshots.
    void append(long senderId, const KinematicState& state)
    {
        auto& h = m_history[senderId];
        h.push_back(state);
        while (h.size() > m_cfg.historyWindow) {
            h.pop_front();
        }
    }

    IMotionModel* model() const { return m_model.get(); }

    void reset() { m_history.clear(); }

private:
    std::map<long, std::deque<KinematicState>> m_history;
    std::unique_ptr<IMotionModel> m_model;
    EngineConfig m_cfg;
};

#endif // PREDICTOR_H_
