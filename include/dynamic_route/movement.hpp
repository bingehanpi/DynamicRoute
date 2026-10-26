#pragma once

#include "dynamic_route/grid_map.hpp"

#include <vector>

namespace dynamic_route {

enum class Connectivity { Four, Eight };

// Returns unblocked neighbors in up, down, left, right order.
// A blocked origin has no neighbors. An out-of-bounds origin throws
// std::out_of_range.
[[nodiscard]] std::vector<Position> four_connected_neighbors(
    const GridMap& map, Position origin);

// Returns four-connected neighbors first, followed by upper-left, upper-right,
// lower-left, lower-right. Diagonals require both adjacent side cells to be
// unblocked. Origin validation matches four_connected_neighbors.
[[nodiscard]] std::vector<Position> eight_connected_neighbors(
    const GridMap& map, Position origin);

// Cost is the destination's terrain weight times step length (1 or sqrt(2)).
// Diagonals are allowed only with Eight and require both side cells unblocked.
// Out-of-bounds endpoints throw std::out_of_range; blocked endpoints, forbidden
// diagonals, stationary moves, and non-adjacent moves throw std::invalid_argument.
[[nodiscard]] double movement_cost(
    const GridMap& map, Position from, Position to,
    Connectivity connectivity = Connectivity::Four);

}  // namespace dynamic_route
