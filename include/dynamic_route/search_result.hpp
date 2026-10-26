#pragma once

#include "dynamic_route/position.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace dynamic_route {

struct SearchStats {
    // Number of times a node's outgoing moves are examined.
    std::size_t expanded_nodes{0};
    // Includes initial entries and repeated entries for the same position.
    std::size_t frontier_pushes{0};
};

struct SearchResult {
    bool found{false};
    // On success, ordered from start to goal, including both endpoints.
    // A failed search leaves the path empty and total_cost at infinity.
    std::vector<Position> path;
    double total_cost{std::numeric_limits<double>::infinity()};
    SearchStats stats;
};

}  // namespace dynamic_route
