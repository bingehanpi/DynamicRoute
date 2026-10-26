#pragma once

#include "dynamic_route/movement.hpp"
#include "dynamic_route/search_result.hpp"

namespace dynamic_route {

// Finds a minimum-cost path using the selected connectivity (Four by default).
// Each move costs its length times the destination terrain weight. Diagonal
// length is sqrt(2), and both side cells must be unblocked. Start is not charged.
// Throws std::out_of_range if either endpoint is outside the map, checked
// before obstacle state. Throws std::invalid_argument for blocked endpoints.
// An unblocked start == goal succeeds with a one-position path, zero cost,
// and zero statistics. An unreachable goal returns found == false, an empty
// path, and infinite cost. Statistics include work done by unsuccessful searches.
[[nodiscard]] SearchResult dijkstra(
    const GridMap& map, Position start, Position goal,
    Connectivity connectivity = Connectivity::Four);

}  // namespace dynamic_route
