#ifndef OCCLUSION_H_
#define OCCLUSION_H_

#include "Geodesy.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace artery {

/**
 * @brief Line-of-sight oracle used before turning "nobody saw it" into negative evidence.
 *
 * A missing detection only counts against a claim when the viewer covered the location
 * AND had a clear line of sight to it. Occluded locations are Unverifiable.
 */
class IOcclusionModel {
public:
    virtual ~IOcclusionModel() = default;
    virtual bool blocked(const GeoPoint& from, const GeoPoint& to) const = 0;
};

class NoOcclusion : public IOcclusionModel {
public:
    bool blocked(const GeoPoint&, const GeoPoint&) const override { return false; }
};

namespace Planar {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline double cross(const Vec2& o, const Vec2& a, const Vec2& b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline double distanceToSegment(const Vec2& a, const Vec2& b, const Vec2& p)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
}

inline bool segmentsIntersect(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2)
{
    const double d1 = cross(q1, q2, p1);
    const double d2 = cross(q1, q2, p2);
    const double d3 = cross(p1, p2, q1);
    const double d4 = cross(p1, p2, q2);
    return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

inline bool pointInPolygon(const Vec2& p, const std::vector<Vec2>& polygon)
{
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[j];
        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

/// True if segment ab crosses the polygon boundary or lies inside it.
inline bool segmentHitsPolygon(const Vec2& a, const Vec2& b, const std::vector<Vec2>& polygon)
{
    if (polygon.size() < 3) {
        return false;
    }
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if (segmentsIntersect(a, b, polygon[j], polygon[i])) {
            return true;
        }
    }
    return pointInPolygon(a, polygon) || pointInPolygon(b, polygon);
}

/// True if segment ab passes through a disc; discs containing an endpoint are the viewer or target itself.
inline bool segmentHitsDisc(const Vec2& a, const Vec2& b, const Vec2& centre, double radius)
{
    if (std::hypot(centre.x - a.x, centre.y - a.y) <= radius || std::hypot(centre.x - b.x, centre.y - b.y) <= radius) {
        return false;
    }
    return distanceToSegment(a, b, centre) < radius;
}

}  // namespace Planar

}  // namespace artery

#endif  // OCCLUSION_H_
