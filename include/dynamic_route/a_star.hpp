#pragma once

#include "dynamic_route/heuristic.hpp"
#include "dynamic_route/movement.hpp"
#include "dynamic_route/search_result.hpp"

namespace dynamic_route {

// A* with the same connectivity, endpoint, cost, and result conventions as
// dijkstra. Nonzero estimates are scaled by the minimum unblocked terrain
// weight, recomputed on each call so weights below 1.0 remain valid. Zero
// skips this scan. All three choices are valid for either connectivity.
// Out-of-bounds endpoints throw std::out_of_range before checking obstacles;
// blocked endpoints throw std::invalid_argument. Start == goal returns zero
// cost and statistics. Unreachable goals retain statistics but have no path.
[[nodiscard]] SearchResult a_star(
    const GridMap& map, Position start, Position goal,
    Connectivity connectivity = Connectivity::Four,
    Heuristic heuristic_type = Heuristic::Automatic);

}  // namespace dynamic_route
