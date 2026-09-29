#ifndef ARTERY_ADAPTERS_H_
#define ARTERY_ADAPTERS_H_

/**
 * Glue between Artery/Vanetza and the OMNeT-free trust core:
 *   - CAM / CPM parsing into claims (vehicle centre, true-north headings, generation times)
 *   - calibration of the SUMO projected grid against WGS-84
 *   - Artery Position <-> WGS-84 mapping around the receiver
 *   - building occlusion from the GlobalEnvironmentModel
 */

#include "../../include/CpmTypes.h"
#include "../../include/Geodesy.h"
#include "../../include/Occlusion.h"
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObstacle.h>
#include <artery/traci/Cast.h>
#include <artery/utility/Geometry.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <traci/API.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <cmath>
#include <cstdint>
#include <string>

namespace artery {

/**
 * How CPM perceived objects are encoded.
 *
 *   Artery  what artery::CpService actually sends (deviates from ETSI TS 103 324 V2.1.1):
 *             - position = (objectPos - egoPos) in GRID metres, +x east, +y SOUTH, grid north
 *             - velocityDirection = heading clockwise from grid north in RADIANS * 10
 *             - positions are taken at referenceTime; its measurementDeltaTime has the opposite sign
 *   Etsi    TS 103 324 V2.1.1 (ASN.1 CPM-PerceivedObjectContainer, ETSI-ITS-CDD):
 *             - objects "in the coordinate system in which the y-axis corresponds to the North direction,
 *               the x-axis to the East direction" (true north, relative to the reference position)
 *             - velocityDirection: CartesianAngle in 0.1 degree, counted counter-clockwise from the x-axis (East)
 *             - measurementDeltaTime: object time = referenceTime + measurementDeltaTime (negative = before)
 */
enum class CpmEncoding { Artery, Etsi };

inline CpmEncoding parseCpmEncoding(const std::string& name)
{
    return (name == "etsi") ? CpmEncoding::Etsi : CpmEncoding::Artery;
}

/// Reception time in simulation seconds and the same instant in TAI milliseconds.
struct MessageTiming {
    double now = 0.0;
    uint64_t nowTaiMs = 0;
};

namespace MessageParsers {

/// CAM generationDeltaTime is TAI ms modulo 65536.
inline double camGenerationTime(long generationDeltaTime, const MessageTiming& timing)
{
    const uint16_t nowMod = static_cast<uint16_t>(timing.nowTaiMs % 65536);
    const uint16_t age = static_cast<uint16_t>(nowMod - static_cast<uint16_t>(generationDeltaTime));
    return timing.now - age / 1000.0;
}

inline CamClaim parseCam(const vanetza::asn1::Cam& cam, const MessageTiming& timing, const GridFrame& grid,
                         double defaultLength)
{
    CamClaim claim;
    claim.author = cam->header.stationID;
    KinematicState& s = claim.state;

    const auto& basic = cam->cam.camParameters.basicContainer;
    const GeoPoint front{basic.referencePosition.latitude * 1e-7, basic.referencePosition.longitude * 1e-7};

    const auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    if (hfc.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        const auto& hf = hfc.choice.basicVehicleContainerHighFrequency;
        const long speed = hf.speed.speedValue;
        const long heading = hf.heading.headingValue;
        const long accel = hf.longitudinalAcceleration.longitudinalAccelerationValue;
        const long yaw = hf.yawRate.yawRateValue;
        const long length = hf.vehicleLength.vehicleLengthValue;
        s.speed = (speed == SpeedValue_unavailable) ? 0.0 : speed * 0.01;
        s.heading = grid.toTrueHeading(heading == HeadingValue_unavailable ? 0.0 : heading * 0.1);
        s.accel = (accel == LongitudinalAccelerationValue_unavailable) ? 0.0 : accel * 0.1;
        // ETSI yaw rate is positive to the LEFT (counter-clockwise); headings grow clockwise.
        s.yawRate = (yaw == YawRateValue_unavailable) ? 0.0 : -yaw * 0.01 * ConstantsWGS84::DEG_TO_RAD;
        claim.length = (length >= VehicleLengthValue_outOfRange) ? 0.0 : length * 0.1;
    }
    s.time = camGenerationTime(cam->cam.generationDeltaTime, timing);

    // CAM reference point is the front bumper; CPM objects are vehicle centres.
    const double len = claim.length > 0.0 ? claim.length : defaultLength;
    const GeoPoint centre = Geodesy::offset(front, s.heading, -0.5 * len);
    s.lat = centre.lat;
    s.lon = centre.lon;
    return claim;
}

inline CpmSnapshot parseCpm(const vanetza::asn1::r2::Cpm& cpm, const MessageTiming& timing, const GridFrame& grid,
                            CpmEncoding encoding)
{
    CpmSnapshot snapshot;
    snapshot.author = cpm->header.stationId;

    const auto& mc = cpm->payload.managementContainer;
    snapshot.referencePosition = GeoPoint{mc.referencePosition.latitude * 1e-7, mc.referencePosition.longitude * 1e-7};
    snapshot.time = timing.now;
    uint64_t referenceTai = 0;
    if (asn_INTEGER2uint64(&mc.referenceTime, &referenceTai) == 0) {
        snapshot.referenceTaiMs = referenceTai;
        if (referenceTai <= timing.nowTaiMs && timing.nowTaiMs - referenceTai < 10000) {
            snapshot.time = timing.now - (timing.nowTaiMs - referenceTai) / 1000.0;
        }
    }

    const Vanetza_ITS2_PerceivedObjectContainer_t* poc = nullptr;
    const auto& containers = cpm->payload.cpmContainers;
    for (int i = 0; i < containers.list.count; ++i) {
        const auto* wcc = containers.list.array[i];
        if (!wcc) continue;
        if (wcc->containerId == Vanetza_ITS2_CpmContainerId_originatingVehicleContainer &&
            wcc->containerData.present ==
                Vanetza_ITS2_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer) {
            const long angle = wcc->containerData.choice.OriginatingVehicleContainer.orientationAngle.value;
            // Artery writes grid-north decidegrees; the standard (Wgs84Angle) is true north already.
            if (angle >= 0 && angle < 3600) {
                snapshot.heading = encoding == CpmEncoding::Artery ? grid.toTrueHeading(angle * 0.1) : angle * 0.1;
            }
        } else if (wcc->containerId == Vanetza_ITS2_CpmContainerId_perceivedObjectContainer &&
                   wcc->containerData.present ==
                       Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer) {
            poc = &wcc->containerData.choice.PerceivedObjectContainer;
        }
    }
    if (!poc) {
        return snapshot;
    }

    snapshot.objects.reserve(poc->perceivedObjects.list.count);
    for (int j = 0; j < poc->perceivedObjects.list.count; ++j) {
        const auto* po = poc->perceivedObjects.list.array[j];
        if (!po) continue;
        ObjectClaim o;
        o.author = snapshot.author;
        o.objectId = po->objectId ? static_cast<uint16_t>(*po->objectId) : static_cast<uint16_t>(j);
        const bool artery = (encoding == CpmEncoding::Artery);
        o.time = artery ? snapshot.time : snapshot.time + po->measurementDeltaTime / 1000.0;

        const double x = po->position.xCoordinate.value * 0.01;
        const double y = po->position.yCoordinate.value * 0.01;
        const LocalPoint offset = artery ? grid.fromArteryDelta(x, y) : LocalPoint{x, y};  // ETSI: east, north
        o.position = Geodesy::fromLocal(snapshot.referencePosition, offset);

        if (po->velocity && po->velocity->present == Vanetza_ITS2_Velocity3dWithConfidence_PR_polarVelocity) {
            const auto& polar = po->velocity->choice.polarVelocity;
            const long speedValue = polar.velocityMagnitude.speedValue;
            const long direction = polar.velocityDirection.value;
            // sentinels (unavailable speed 16383, unavailable angle 3601) mean "no velocity", not 163.83 m/s
            if (speedValue != Vanetza_ITS2_SpeedValue_unavailable && direction >= 0 &&
                (artery || direction < 3600)) {
                o.speed = speedValue * 0.01;
                if (artery) {
                    const double gridHeadingDeg = direction * 0.1 * ConstantsWGS84::RAD_TO_DEG;
                    o.heading = grid.toTrueHeading(gridHeadingDeg);
                } else {
                    o.heading = Geodesy::wrap360(90.0 - direction * 0.1);  // CCW from east -> CW from north
                }
                o.hasVelocity = true;
            }
        }
        snapshot.objects.push_back(o);
    }
    return snapshot;
}

}  // namespace MessageParsers

/// Measures meridian convergence and scale of the SUMO projection at a point (two TraCI calls).
inline GridFrame calibrateGrid(const traci::API& api, const traci::TraCIPosition& sumoAnchor)
{
    traci::TraCIPosition north = sumoAnchor;
    north.y += 100.0;  // SUMO y grows toward grid north
    const traci::TraCIGeoPosition g0 = api.convertGeo(sumoAnchor);
    const traci::TraCIGeoPosition g1 = api.convertGeo(north);
    const LocalPoint d = Geodesy::toLocal(GeoPoint{g0.latitude, g0.longitude}, GeoPoint{g1.latitude, g1.longitude});
    GridFrame frame;
    frame.northBearingDeg = Geodesy::wrap180(Geodesy::bearing(d));
    frame.scale = Geodesy::norm(d) / 100.0;
    return frame;
}

/// Maps Artery grid positions to WGS-84 around an anchor known in both systems.
struct ArteryGeoMapper {
    GridFrame grid;
    Position anchor;
    GeoPoint anchorGeo;

    GeoPoint toGeo(const Position& p) const
    {
        const LocalPoint d = grid.fromArteryDelta(p.x.value() - anchor.x.value(), p.y.value() - anchor.y.value());
        return Geodesy::fromLocal(anchorGeo, d);
    }

    Position toArtery(const GeoPoint& g) const
    {
        const LocalPoint d = grid.toArteryDelta(Geodesy::toLocal(anchorGeo, g));
        return Position(anchor.x.value() + d.east, anchor.y.value() + d.north);
    }
};

/// Line of sight blocked by buildings known to the GlobalEnvironmentModel.
class BuildingOcclusion : public IOcclusionModel {
public:
    BuildingOcclusion(GlobalEnvironmentModel* gem, const ArteryGeoMapper& mapper) : m_gem(gem), m_mapper(mapper) {}

    bool blocked(const GeoPoint& from, const GeoPoint& to) const override
    {
        if (!m_gem) {
            return false;
        }
        const Position a = m_mapper.toArtery(from);
        const Position b = m_mapper.toArtery(to);
        const double minX = std::min(a.x.value(), b.x.value()) - 1.0;
        const double maxX = std::max(a.x.value(), b.x.value()) + 1.0;
        const double minY = std::min(a.y.value(), b.y.value()) - 1.0;
        const double maxY = std::max(a.y.value(), b.y.value()) + 1.0;
        std::vector<Position> area{Position(minX, minY), Position(maxX, minY), Position(maxX, maxY),
                                   Position(minX, maxY)};
        boost::geometry::correct(area);

        const Planar::Vec2 pa{a.x.value(), a.y.value()};
        const Planar::Vec2 pb{b.x.value(), b.y.value()};
        for (const auto& obstacle : m_gem->preselectObstacles(area)) {
            std::vector<Planar::Vec2> polygon;
            polygon.reserve(obstacle->getOutline().size());
            for (const Position& p : obstacle->getOutline()) {
                polygon.push_back(Planar::Vec2{p.x.value(), p.y.value()});
            }
            if (Planar::segmentHitsPolygon(pa, pb, polygon)) {
                return true;
            }
        }
        return false;
    }

private:
    GlobalEnvironmentModel* m_gem;
    ArteryGeoMapper m_mapper;
};

}  // namespace artery

#endif  // ARTERY_ADAPTERS_H_
