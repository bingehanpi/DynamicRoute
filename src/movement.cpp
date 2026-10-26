#include "dynamic_route/movement.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace dynamic_route {

std::vector<Position> four_connected_neighbors(const GridMap& map,
                                              const Position origin) {
    // is_blocked also checks that origin is inside the map.
    if (map.is_blocked(origin)) {
        return {};
    }

    constexpr Position offsets[] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    std::vector<Position> neighbors;
    neighbors.reserve(4);

    for (const Position offset : offsets) {
        const Position candidate{origin.row + offset.row,
                                 origin.column + offset.column};
        if (map.contains(candidate) && !map.is_blocked(candidate)) {
            neighbors.push_back(candidate);
        }
    }

    return neighbors;
}

std::vector<Position> eight_connected_neighbors(const GridMap& map,
                                               const Position origin) {
    // This also validates the origin and preserves the four-neighbor order.
    std::vector<Position> neighbors = four_connected_neighbors(map, origin);
    if (map.is_blocked(origin)) {
        return neighbors;
    }
    neighbors.reserve(8);
    constexpr Position offsets[] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    for (const Position offset : offsets) {
        const Position candidate{origin.row + offset.row,
                                 origin.column + offset.column};
        if (!map.contains(candidate) || map.is_blocked(candidate)) {
            continue;
        }
        // For a valid diagonal destination, both side cells are in bounds.
        if (!map.is_blocked(Position{origin.row, candidate.column}) &&
            !map.is_blocked(Position{candidate.row, origin.column})) {
            neighbors.push_back(candidate);
        }
    }
    return neighbors;
}

double movement_cost(const GridMap& map, const Position from, const Position to,
                     const Connectivity connectivity) {
    if (!map.contains(from) || !map.contains(to)) {
        throw std::out_of_range("Movement endpoints must be inside GridMap");
    }
    if (map.is_blocked(from) || map.is_blocked(to)) {
        throw std::invalid_argument("Movement endpoints must be unblocked");
    }

    const int row_distance = std::abs(to.row - from.row);
    const int column_distance = std::abs(to.column - from.column);
    const bool straight = (row_distance == 1 && column_distance == 0) ||
                          (row_distance == 0 && column_distance == 1);
    if (straight) {
        return map.terrain_weight(to);
    }
    if (connectivity == Connectivity::Eight && row_distance == 1 &&
        column_distance == 1) {
        if (map.is_blocked(Position{from.row, to.column}) ||
            map.is_blocked(Position{to.row, from.column})) {
            throw std::invalid_argument("Diagonal movement cannot cross obstacle corners");
        }
        return std::sqrt(2.0) * map.terrain_weight(to);
    }
    throw std::invalid_argument("Movement must be one permitted adjacent step");
}

}  // namespace dynamic_route
