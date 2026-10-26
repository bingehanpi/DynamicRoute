#include "dynamic_route/a_star.hpp"

#include "dynamic_route/movement.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace dynamic_route {

SearchResult a_star(const GridMap& map, const Position start,
                    const Position goal, const Connectivity connectivity,
                    const Heuristic heuristic_type) {
    if (!map.contains(start) || !map.contains(goal)) {
        throw std::out_of_range("Search endpoints must be inside GridMap");
    }
    if (map.is_blocked(start) || map.is_blocked(goal)) {
        throw std::invalid_argument("Search endpoints must be unblocked");
    }
    if (start == goal) {
        return {true, {start}, 0.0, {}};
    }

    const double minimum_weight = heuristic_type == Heuristic::Zero
                                      ? 0.0
                                      : minimum_unblocked_terrain_weight(map);
    // Ignore obstacles and use the cheapest terrain to get a lower bound.
    const auto heuristic = [goal, minimum_weight, connectivity,
                            heuristic_type](const Position position) {
        return heuristic_distance(position, goal, connectivity, heuristic_type) *
               minimum_weight;
    };

    const auto columns = static_cast<std::size_t>(map.column_count());
    const auto cell_count = static_cast<std::size_t>(map.row_count()) * columns;
    const auto index_of = [columns](const Position position) {
        return static_cast<std::size_t>(position.row) * columns +
               static_cast<std::size_t>(position.column);
    };
    const auto position_of = [columns](const std::size_t index) {
        return Position{static_cast<int>(index / columns),
                        static_cast<int>(index % columns)};
    };

    std::vector<double> distances(cell_count,
                                  std::numeric_limits<double>::infinity());
    std::vector<std::size_t> parents(cell_count);
    // Entries sort by f = g + h, then g, then cell index for deterministic ties.
    using Entry = std::tuple<double, double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
    const std::size_t start_index = index_of(start);
    distances[start_index] = 0.0;
    frontier.emplace(heuristic(start), 0.0, start_index);

    SearchResult result;
    result.stats.frontier_pushes = 1;
    while (!frontier.empty()) {
        const auto [estimated_total, cost, index] = frontier.top();
        frontier.pop();
        // Check the actual cost g, not the estimated total f.
        if (cost != distances[index]) {
            continue;
        }

        const Position current = position_of(index);
        if (current == goal) {
            result.found = true;
            result.total_cost = cost;
            for (std::size_t step = index;; step = parents[step]) {
                result.path.push_back(position_of(step));
                if (step == start_index) {
                    break;
                }
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        ++result.stats.expanded_nodes;
        const auto neighbors = connectivity == Connectivity::Eight
                                   ? eight_connected_neighbors(map, current)
                                   : four_connected_neighbors(map, current);
        for (const Position neighbor : neighbors) {
            const std::size_t neighbor_index = index_of(neighbor);
            const double next_cost =
                cost + movement_cost(map, current, neighbor, connectivity);
            if (next_cost < distances[neighbor_index]) {
                distances[neighbor_index] = next_cost;
                parents[neighbor_index] = index;
                frontier.emplace(next_cost + heuristic(neighbor), next_cost,
                                 neighbor_index);
                ++result.stats.frontier_pushes;
            }
        }
    }
    return result;
}

}  // namespace dynamic_route
