#pragma once

namespace veins {
    struct Coord {
        double x, y, z;
        Coord(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    };
}
