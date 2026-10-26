#include "dynamic_route/heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dynamic_route {

double minimum_unblocked_terrain_weight(const GridMap& map) {
    double minimum = std::numeric_limits<double>::infinity();
    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            const Position position{row, column};
            if (!map.is_blocked(position)) {
                minimum = std::min(minimum, map.terrain_weight(position));
            }
        }
    }
    return minimum;
}

double heuristic_distance(const Position from, const Position to,
                          const Connectivity connectivity,
                          const Heuristic heuristic_type) noexcept {
    if (heuristic_type == Heuristic::Zero) {
        return 0.0;
    }
    // Convert before subtraction so even extreme integer coordinates are safe.
    const double rows = std::abs(static_cast<double>(from.row) - to.row);
    const double columns = std::abs(static_cast<double>(from.column) - to.column);
    if (heuristic_type == Heuristic::Euclidean) {
        return std::hypot(rows, columns);
    }
    if (connectivity == Connectivity::Eight) {
        const double diagonal_steps = std::min(rows, columns);
        const double straight_steps = std::max(rows, columns) - diagonal_steps;
        return std::sqrt(2.0) * diagonal_steps + straight_steps;
    }
    return rows + columns;
}

}  // namespace dynamic_route
