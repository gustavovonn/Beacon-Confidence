#ifndef OT_SHARED_GEODESY_H_
#define OT_SHARED_GEODESY_H_

/**
 * Local tangent plane on the WGS-84 ellipsoid.
 *
 * Lifted from cp-trustfulness/include/{Geodesy.h, Constants.h} (toLocal, the two radii of curvature
 * and the WGS-84 parameters), reduced to what this experiment needs and renamed into this
 * namespace. Copied rather than included so the engine, the parser and the cost evaluator build on
 * the VisionFive 2 with nothing but a compiler. If the original changes, this does not follow.
 *
 * Everything past the parser is metres in this plane. Latitude and longitude appear exactly once,
 * at ingestion, and never again.
 */

#include <cmath>

namespace ot {
namespace geo {

constexpr double kWgs84A = 6378137.0;          ///< semi-major axis [m]
constexpr double kWgs84E2 = 0.00669437999014;  ///< first eccentricity squared, e^2 = 2f - f^2
constexpr double kDegToRad = M_PI / 180.0;

/// Meridional (north-south) radius of curvature at a latitude [m].
inline double meridionalRadius(double latRad)
{
    const double s = std::sin(latRad);
    const double w = std::sqrt(1.0 - kWgs84E2 * s * s);
    return kWgs84A * (1.0 - kWgs84E2) / (w * w * w);
}

/// Prime-vertical (east-west) radius of curvature at a latitude [m].
inline double primeVerticalRadius(double latRad)
{
    const double s = std::sin(latRad);
    const double w = std::sqrt(1.0 - kWgs84E2 * s * s);
    return kWgs84A / w;
}

struct GeoPoint {
    double lat = 0.0;   ///< [deg]
    double lon = 0.0;   ///< [deg]
};

struct LocalPoint {
    double east = 0.0;   ///< [m]
    double north = 0.0;  ///< [m]
};

/// Displacement of `p` from `origin`, in metres on the tangent plane at `origin`.
inline LocalPoint toLocal(const GeoPoint& origin, const GeoPoint& p)
{
    const double latRad = origin.lat * kDegToRad;
    LocalPoint d;
    d.north = (p.lat - origin.lat) * kDegToRad * meridionalRadius(latRad);
    d.east = (p.lon - origin.lon) * kDegToRad * primeVerticalRadius(latRad) * std::cos(latRad);
    return d;
}

}  // namespace geo
}  // namespace ot

#endif  // OT_SHARED_GEODESY_H_
