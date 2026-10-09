#ifndef CONFIDENCE_H_
#define CONFIDENCE_H_

/**
 * Vocabulary of the confidence model: Paper 1 applied to CAM and CPM motion vectors, plus corroboration.
 *
 *   Claim stream  the successive motion vectors of one origin about one vehicle: a CAM sender's own trajectory
 *                 (station id), or one author's reports of one perceived object ((author, CPM object id)).
 *                 The key is provenance: it says who claimed what, never which body the receiver thinks
 *                 it is. Plausibility is judged INSIDE a stream, where the author's positioning bias is on
 *                 both sides of the comparison and cancels.
 *   Confidence    S, one per claim stream: Paper 1's counterless Beta score, fed by the evidence of the checks (for
 *                 or against) and by corroboration from other stations on the same object hypothesis (only
 *                 for). It is not a sender reputation, not an object-existence estimate and not a
 *                 calibrated probability, and it is not comparable across streams of different maturity:
 *                 a fresh stream sits at the uninformed prior, above the honest steady state.
 *   State         a vector is implausible (flagged) when its stream's confidence is below 0.5, otherwise plausible.
 */

#include <cstdint>

namespace artery {

using StationId = long;

enum class ClaimKind { Cam, Object };

enum class ClaimState { Plausible, Implausible };

/**
 * Which check an implausible state is owed to. The vocabulary follows the misbehaviour-detection
 * literature, where the dividing line is how many messages a check needs:
 *
 *   PLAUSIBILITY  one message against an absolute or environmental bound — "is the advertised speed
 *                 below a predefined threshold", "is the station in a coherent place" [Kamel et al., CaTch].
 *   CONSISTENCY   across messages, over time from one sender or across senders [van der Heijden et al.,
 *                 IEEE Surveys 2019; LA-DETECTS].
 *
 * Our previous work called the two consistency checks "phase 0" and "phase 1" and grouped them under
 * "plausibility", which inverts the literature: both compare successive vectors of one stream, so both are
 * consistency, while the single-message bounds it called "impossible" are the plausibility checks.
 */
enum class Reason {
    None,
    SpeedPlausibility,      // one message: speed outside [0, maxSpeed], or values that cannot be evaluated
    RangePlausibility,      // one message: a perceived object outside its author's sensing geometry
    PosSpeedConsistency,    // two successive vectors: implied displacement speed vs reported speed
    TrajectoryConsistency,  // the window: OLS-CTRA prediction error vs the manoeuvre budget d_max
    PoseConsistency,        // a CPM's declared pose vs the same station's own CAM stream
};

/**
 * Paper 1's counterless Beta update (Sec. IV-B): pseudo-counts are rebuilt from the previous score with a fixed
 * prior weight, the evidence is added, and the posterior mean is the new score. With no evidence the score
 * drifts back towards 0.5, so a stream that brings no evidence loses its support over time.
 */
inline double counterlessUpdate(double score, double support, double contradiction, double priorWeight)
{
    const double alpha = score * priorWeight + 1.0 + support;
    const double beta = (1.0 - score) * priorWeight + 1.0 + contradiction;
    return alpha / (alpha + beta);
}

inline const char* toString(ClaimKind kind)
{
    return kind == ClaimKind::Cam ? "cam" : "object";
}

inline const char* toString(Reason reason)
{
    switch (reason) {
        case Reason::None:                  return "none";
        case Reason::SpeedPlausibility:     return "speed_plausibility";
        case Reason::RangePlausibility:     return "range_plausibility";
        case Reason::PosSpeedConsistency:   return "pos_speed_consistency";   // previously "phase 0"
        case Reason::TrajectoryConsistency: return "trajectory_consistency";  // previously "phase 1"
        case Reason::PoseConsistency:       return "pose_consistency";
    }
    return "unknown";
}

inline const char* toString(ClaimState state)
{
    switch (state) {
        case ClaimState::Plausible:   return "plausible";
        case ClaimState::Implausible: return "implausible";
    }
    return "unknown";
}

}  // namespace artery

#endif  // CONFIDENCE_H_
