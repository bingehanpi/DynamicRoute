#pragma once

#include "dynamic_route/heuristic.hpp"
#include "dynamic_route/movement.hpp"
#include "dynamic_route/search_result.hpp"

namespace dynamic_route {

// Bidirectional A*, Four and Automatic by default. Automatic uses Manhattan
// for Four and Octile for Eight; Euclidean uses straight-line distance. Nonzero
// estimates are scaled by minimum unblocked terrain weight. Zero sets h = 0
// in both directions and skips the minimum-weight scan.
// Eight uses the movement layer's diagonal costs and obstacle-corner checks.
// Endpoint checks and result conventions match dijkstra. Backward search uses
// original forward-edge costs. Statistics sum work from both search directions;
// a position expanded in both directions is counted twice. Start == goal has
// zero cost and statistics. Both directions use the selected heuristic type.
[[nodiscard]] SearchResult bidirectional_a_star(
    const GridMap& map, Position start, Position goal,
    Connectivity connectivity = Connectivity::Four,
    Heuristic heuristic_type = Heuristic::Automatic);

}  // namespace dynamic_route
