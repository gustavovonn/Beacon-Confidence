#ifndef CONFIDENCE_H_
#define CONFIDENCE_H_

/**
 * Vocabulary of the confidence model: Paper 1 applied to CAM and CPM content, plus exteroceptive anchors.
 *
 *   Claim track   what successive messages say about one thing: a CAM sender's own trajectory, or one
 *                 perceived object of one CPM author (author, CPM object id).
 *   Plausibility  P, Paper 1's counterless Beta score of the track (physical invariance + CTRA prediction).
 *   Anchoring     A, the same filter fed only by sources the author does not control: the receiver's own
 *                 radar confirms or contradicts; other authors only confirm, with statements on the same LDM track.
 *   State         what one message is worth to an application:
 *                   implausible   P < 0.5         Paper 1 rejects it
 *                   contradicted  A < 0.4         the receiver's radar denies it
 *                   anchored      A >= 0.58       plausible and confirmed by an independent sensor
 *                   unanchored    otherwise       plausible, but nothing independent could check it
 */

#include <cstdint>

namespace artery {

using StationId = long;

enum class ClaimKind { Cam, Object };

enum class Verdict { Supported, Contradicted, Unverifiable };

enum class ClaimState { Anchored, Unanchored, Contradicted, Implausible };

/// Which component a state is owed to (for attribution): the last penalty of P, or the receiver's radar.
enum class Reason { None, Physics, Kinematics, Impossible, Pose, Radar };

/**
 * Paper 1's counterless Beta update (Sec. IV-B): pseudo-counts are rebuilt from the previous score with a fixed
 * prior weight, the evidence is added, and the posterior mean is the new score. With no evidence the score
 * drifts back towards 0.5, so anchoring expires when nothing independent confirms a claim any more.
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

inline const char* toString(Verdict verdict)
{
    switch (verdict) {
        case Verdict::Supported:    return "supported";
        case Verdict::Contradicted: return "contradicted";
        case Verdict::Unverifiable: return "unverifiable";
    }
    return "unknown";
}

inline const char* toString(Reason reason)
{
    switch (reason) {
        case Reason::None:       return "none";
        case Reason::Physics:    return "physics";      // Paper 1 phase 0 (implied vs reported speed, frozen position)
        case Reason::Kinematics: return "kinematics";   // Paper 1 phase 1 (OLS-CTRA prediction)
        case Reason::Impossible: return "impossible";   // an object report that cannot be physical
        case Reason::Pose:       return "pose";         // CPM reference pose vs the author's CAM track
        case Reason::Radar:      return "radar";        // the receiver's radar contradicts
    }
    return "unknown";
}

inline const char* toString(ClaimState state)
{
    switch (state) {
        case ClaimState::Anchored:     return "anchored";
        case ClaimState::Unanchored:   return "unanchored";
        case ClaimState::Contradicted: return "contradicted";
        case ClaimState::Implausible:  return "implausible";
    }
    return "unknown";
}

}  // namespace artery

#endif  // CONFIDENCE_H_
