#ifndef OT_OBJECT_TRACK_H_
#define OT_OBJECT_TRACK_H_

/**
 * One believed object trajectory. This is what the engine publishes and what confidence is attached
 * to -- not a message, not a sender, the trajectory itself.
 *
 * `contributors` is recorded from day one on purpose. It costs almost nothing, the association
 * statistics need it anyway, and without it a disagreement cannot be attributed: one source
 * consistently off looks identical to the whole track being wrong. With it, the two separate.
 */

#include "UnscentedCTRA.h"
#include "MotionVector.h"
#include <cstdint>
#include <map>
#include <vector>

namespace ot {

/// Why a report did not end up fused, so the confidence function can tell the cases apart.
enum class Outcome : uint8_t {
    Fused,          ///< matched a track and agreed with it
    Rejected,       ///< matched a track but disagreed beyond the gate
    StartedTrack,   ///< matched nothing, became a new trajectory
    Dropped,        ///< failed plausibility, never reached association
};

struct ObjectTrack {
    uint64_t id = 0;  // local
    UnscentedCTRA kf;  // estimated state and covariance

    double firstSeen = 0.0;
    double lastUpdate = 0.0;
    int hits = 0;              ///< reports fused
    int misses = 0;            ///< consecutive rounds with no report
    bool confirmed = false;    ///< survived M-of-N

    int objectClass = 0;
    double length = 0.0;
    double width = 0.0;

    /// station -> time it last fed this track. Provenance; never used to judge content.
    std::map<uint64_t, double> contributors;

    /// Evidence counters used by the default confidence policy. Kept on the track rather than
    /// derived from `confidence` because a scalar cannot be un-updated: with a single number, the
    /// value 1.0 is a fixed point of any "add evidence to both sides" rule and the score freezes.
    /// A replacement policy is free to ignore these and write `confidence` directly.
    double alpha = 1.0;       ///< evidence the trajectory is a real object moving as believed
    double beta = 1.0;        ///< evidence against

    /// The published number. Under the default policy this is alpha / (alpha + beta), which starts
    /// at 0.5: one report is not yet a reason to believe anything.
    double confidence = 0.5;

    /// Evaluation only: how often each true station was the source. The runner reads this to decide
    /// whether a track is "really" about one object, and to count identity switches.
    std::map<int64_t, int> trueStationHits;

    double x() const { return kf.state(kCtraPx); }
    double y() const { return kf.state(kCtraPy); }
};

/// One line of evidence about one report, written for every motion vector the engine sees.
struct VectorRecord {
    double t = 0.0;
    uint64_t receiver = 0;
    uint64_t author = 0;
    Origin origin = Origin::Cam;
    int64_t trueStation = -1;   ///< oracle

    uint64_t trackId = 0;       ///< 0 when none
    Outcome outcome = Outcome::Dropped;

    double gap = -1.0;          ///< [m] euclidean, for reading by eye
    double normalisedGap = -1.0;///< squared Mahalanobis: the gap judged against our own doubt
    double runnerUpGap = -1.0;  ///< second-best normalised gap; small margin = contested match
    int gatedCandidates = 0;    ///< k, the number that passed the gate
    int contributors = 0;       ///< distinct stations feeding the matched track
    double confidence = 1.0;
};

}  // namespace ot

#endif  // OT_OBJECT_TRACK_H_
