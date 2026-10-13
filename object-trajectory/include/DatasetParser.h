#ifndef OT_DATASET_PARSER_H_
#define OT_DATASET_PARSER_H_

/**
 * Reads a dataset written by ArteryDatasetGenerator and hands the engine motion vectors.
 *
 * This is where the container stops mattering. A CAM about its own sender and a CPM's perceived
 * object both come out as a MotionVector; past this point nothing downstream can tell which wire
 * format carried it, which is the premise of the whole experiment.
 *
 * Three conversions happen here and nowhere else:
 *   1. Geodetic to metric. Everything after this is metres in one local tangent plane, whose origin
 *      is fixed by the dataset (the first CAM position, recorded in the manifest so a re-parse of
 *      the same dataset produces the same numbers).
 *   2. Degrees to radians. Headings stay clockwise from north, which is what the CTRA state uses.
 *   3. Author-relative to absolute. The dataset stores perceived objects as offsets from their
 *      author precisely so that error injection can act on the two separately; the engine needs
 *      them placed, so the parser composes author pose with offset. An injector that wants to
 *      perturb the author's pose or the sensor reading does so BEFORE this composition.
 *
 * Output is grouped into deliveries: one receiver, one arrival time, and the vectors that arrived
 * together. The grouping is not cosmetic -- a CPM's objects must reach the engine in one call, or
 * the joint assignment degrades into greedy first-fit.
 */

#include "MotionVector.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ot {

/// One message arriving at one receiver: everything it carried, and when it got there.
struct Delivery {
    double tRx = 0.0;          ///< [s] arrival at the receiver
    uint64_t receiver = 0;
    std::string messageId;
    std::vector<MotionVector> vectors;
};

/// What the manifest froze, plus what the parser derived.
struct DatasetInfo {
    std::string path;
    double originLat = 0.0;    ///< tangent-plane origin
    double originLon = 0.0;
    std::size_t cams = 0;
    std::size_t cpms = 0;
    std::size_t objects = 0;
    std::size_t objectsWithOracle = 0;
    std::size_t receptions = 0;
    std::size_t deliveries = 0;
    double firstTime = 0.0;
    double lastTime = 0.0;
};

class DatasetParser {
public:
    /// Reads cam.csv, cpm.csv and reception.csv under `dir`. Returns false and fills `error` if a
    /// required file is missing or a header does not match what this parser expects -- a silently
    /// mis-parsed column would produce plausible nonsense, which is worse than a failure.
    bool load(const std::string& dir, std::string& error);

    /// Every delivery, sorted by arrival time then by receiver. One pass over this in order is a
    /// faithful replay of what the receivers saw.
    const std::vector<Delivery>& deliveries() const { return m_deliveries; }

    const DatasetInfo& info() const { return m_info; }

    /// Deliveries for one receiver, in arrival order. The experiment runner uses this to give each
    /// receiver its own engine instance, which is what a real deployment has.
    std::vector<const Delivery*> forReceiver(uint64_t receiver) const;

    /// Every receiver that appears in reception.csv.
    std::vector<uint64_t> receivers() const;

private:
    std::vector<Delivery> m_deliveries;
    DatasetInfo m_info;
};

}  // namespace ot

#endif  // OT_DATASET_PARSER_H_
