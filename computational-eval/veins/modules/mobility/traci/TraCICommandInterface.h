#pragma once
#include <list>
#include <string>
#include "veins/base/utils/Coord.h"

namespace veins {
    // Mock the lane object
    class TraCILane {
    public:
        std::list<Coord> getShape() {
            return std::list<Coord>(); // Return an empty list of coordinates
        }
    };

    // Expand our mock TraCI interface
    class TraCICommandInterface {
    public:
        TraCICommandInterface() {}
        ~TraCICommandInterface() {}

        // Mock the map data F2MD asks for
        std::list<std::string> getLaneIds() {
            return std::list<std::string>(); // Return no lanes
        }

        TraCILane lane(const std::string& id) {
            return TraCILane(); // Return our mock lane
        }
    };
}