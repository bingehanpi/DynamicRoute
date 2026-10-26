#pragma once

#include "dynamic_route/movement.hpp"

namespace dynamic_route {

// Automatic uses Manhattan for Four and Octile for Eight. Euclidean uses
// straight-line distance. Zero sets h = 0, ordering each frontier by actual cost.
enum class Heuristic { Automatic, Euclidean, Zero };

// Returns infinity if every cell is blocked. Search callers validate their
// endpoints first, guaranteeing at least one unblocked cell.
[[nodiscard]] double minimum_unblocked_terrain_weight(const GridMap& map);

// Returns an unweighted geometric lower bound on path length. Coordinates
// need not belong to a map. Multiply by minimum unblocked terrain weight to
// estimate cost; Zero returns 0 for either connectivity.
[[nodiscard]] double heuristic_distance(Position from, Position to,
                                        Connectivity connectivity,
                                        Heuristic heuristic_type) noexcept;

}  // namespace dynamic_route
