#ifndef MESSAGE_EDITING_H_
#define MESSAGE_EDITING_H_

/// Helpers to modify outgoing CAM/CPM ASN.1 objects in place (noise and attack injection).

#include "../../include/Geodesy.h"
#include <vanetza/asn1/cpm.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace artery {
namespace MessageEditing {

/// Moves a reference position (ETSI units of 0.1 microdegree) by metric east/north offsets.
inline void shiftReference(long& latitude, long& longitude, double east, double north)
{
    const GeoPoint from{latitude * 1e-7, longitude * 1e-7};
    const GeoPoint to = Geodesy::fromLocal(from, LocalPoint{east, north});
    latitude = std::lround(to.lat * 1e7);
    longitude = std::lround(to.lon * 1e7);
}

inline Vanetza_ITS2_PerceivedObjectContainer_t* perceivedObjects(vanetza::asn1::r2::Cpm& cpm, bool create = false)
{
    auto& containers = cpm->payload.cpmContainers.list;
    for (int i = 0; i < containers.count; ++i) {
        auto* wcc = containers.array[i];
        if (wcc && wcc->containerId == Vanetza_ITS2_CpmContainerId_perceivedObjectContainer &&
            wcc->containerData.present == Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer) {
            return &wcc->containerData.choice.PerceivedObjectContainer;
        }
    }
    if (!create) {
        return nullptr;
    }
    auto* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
    wcc->containerId = Vanetza_ITS2_CpmContainerId_perceivedObjectContainer;
    wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    auto* poc = &wcc->containerData.choice.PerceivedObjectContainer;
    poc->numberOfPerceivedObjects = 0;
    ASN_SEQUENCE_ADD(&cpm->payload.cpmContainers.list, wcc);
    return poc;
}

/// Keeps only the perceived objects for which `keep` returns true; frees the others.
inline void filterObjects(Vanetza_ITS2_PerceivedObjectContainer_t& poc,
                          const std::function<bool(Vanetza_ITS2_PerceivedObject_t&)>& keep)
{
    auto& list = poc.perceivedObjects.list;
    int kept = 0;
    for (int j = 0; j < list.count; ++j) {
        Vanetza_ITS2_PerceivedObject_t* po = list.array[j];
        if (po && keep(*po)) {
            list.array[kept++] = po;
        } else if (po) {
            ASN_STRUCT_FREE(asn_DEF_Vanetza_ITS2_PerceivedObject, po);
        }
    }
    list.count = kept;
    poc.numberOfPerceivedObjects = kept;
}

/// Sorts perceived objects by position (then id). Artery lists them in an order that depends on heap addresses;
/// every per-object random draw must see a canonical order or runs are not reproducible.
inline void canonicalOrder(Vanetza_ITS2_PerceivedObjectContainer_t& poc)
{
    auto& list = poc.perceivedObjects.list;
    std::sort(list.array, list.array + list.count,
              [](const Vanetza_ITS2_PerceivedObject_t* a, const Vanetza_ITS2_PerceivedObject_t* b) {
                  if (!a || !b) return a != nullptr && b == nullptr;
                  const long ax = a->position.xCoordinate.value, bx = b->position.xCoordinate.value;
                  if (ax != bx) return ax < bx;
                  const long ay = a->position.yCoordinate.value, by = b->position.yCoordinate.value;
                  if (ay != by) return ay < by;
                  const long ai = a->objectId ? *a->objectId : -1, bi = b->objectId ? *b->objectId : -1;
                  return ai < bi;
              });
}

/// Clamps a CartesianCoordinateLarge value (centimetres) to its valid range.
inline long clampCoordinate(long cm)
{
    return std::max<long>(Vanetza_ITS2_CartesianCoordinateLarge_negativeOutOfRange,
                          std::min<long>(Vanetza_ITS2_CartesianCoordinateLarge_positiveOutOfRange, cm));
}

}  // namespace MessageEditing
}  // namespace artery

#endif  // MESSAGE_EDITING_H_
