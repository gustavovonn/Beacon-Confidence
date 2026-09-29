#ifndef EVALUATION_TRUTH_H_
#define EVALUATION_TRUTH_H_

/**
 * EVALUATION ONLY. Which transmitted CPM content was falsified: a whole CPM (its reference position, and so every
 * object it places, or every object shifted) or single objects (a ghost), and which vehicle each perceived object
 * really is. Senders record it; receivers look it up to label what they receive. The confidence engine never sees it.
 */

#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <iterator>
#include <utility>
#include <vanetza/asn1/cpm.hpp>

namespace artery {
namespace EvaluationTruth {

using CpmKey = std::pair<long, uint64_t>;                ///< (station id, CPM referenceTime in TAI ms)
using ObjectKey = std::tuple<long, uint64_t, uint16_t>;  ///< + CPM object id

inline std::set<CpmKey>& falsifiedCpms()
{
    static std::set<CpmKey> registry;
    return registry;
}

inline std::set<ObjectKey>& falsifiedObjects()
{
    static std::set<ObjectKey> registry;
    return registry;
}

inline uint64_t referenceTime(const vanetza::asn1::r2::Cpm& cpm)
{
    uint64_t tai = 0;
    asn_INTEGER2uint64(&cpm->payload.managementContainer.referenceTime, &tai);
    return tai;
}

/// Every object of this CPM is falsified.
inline void markFalsified(const vanetza::asn1::r2::Cpm& cpm)
{
    falsifiedCpms().insert(CpmKey{static_cast<long>(cpm->header.stationId), referenceTime(cpm)});
}

/// One object of this CPM is falsified.
inline void markFalsifiedObject(const vanetza::asn1::r2::Cpm& cpm, uint16_t objectId)
{
    falsifiedObjects().insert(ObjectKey{static_cast<long>(cpm->header.stationId), referenceTime(cpm), objectId});
}

/// Which vehicle (station id) a perceived object really is, recorded by the author before any error is added;
/// absent for objects that are no vehicle with a station id (e.g. a ghost).
inline std::map<ObjectKey, long>& objectStations()
{
    static std::map<ObjectKey, long> registry;
    return registry;
}

inline void setObjectStation(const vanetza::asn1::r2::Cpm& cpm, uint16_t objectId, long station)
{
    objectStations()[ObjectKey{static_cast<long>(cpm->header.stationId), referenceTime(cpm), objectId}] = station;
}

/// The true station of an object, -1 if unknown.
inline long objectStation(long author, uint64_t referenceTai, uint16_t objectId)
{
    auto it = objectStations().find(ObjectKey{author, referenceTai, objectId});
    return it == objectStations().end() ? -1 : it->second;
}

/// Forgets CPMs older than `referenceTai` (checked at most once per simulated second).
inline void evictBefore(uint64_t referenceTai)
{
    static uint64_t lastEviction = 0;
    if (referenceTai < lastEviction + 1000) {
        return;
    }
    lastEviction = referenceTai;
    for (auto it = falsifiedCpms().begin(); it != falsifiedCpms().end();) {
        it = it->second < referenceTai ? falsifiedCpms().erase(it) : std::next(it);
    }
    for (auto it = falsifiedObjects().begin(); it != falsifiedObjects().end();) {
        it = std::get<1>(*it) < referenceTai ? falsifiedObjects().erase(it) : std::next(it);
    }
    for (auto it = objectStations().begin(); it != objectStations().end();) {
        it = std::get<1>(it->first) < referenceTai ? objectStations().erase(it) : std::next(it);
    }
}

inline bool isFalsified(long station, uint64_t referenceTai)
{
    return falsifiedCpms().count(CpmKey{station, referenceTai}) > 0;
}

inline bool isFalsifiedObject(long station, uint64_t referenceTai, uint16_t objectId)
{
    return isFalsified(station, referenceTai) || falsifiedObjects().count(ObjectKey{station, referenceTai, objectId}) > 0;
}

}  // namespace EvaluationTruth
}  // namespace artery

#endif  // EVALUATION_TRUTH_H_
