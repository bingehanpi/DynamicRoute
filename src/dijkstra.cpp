#include "dynamic_route/dijkstra.hpp"

#include "dynamic_route/movement.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dynamic_route {

SearchResult dijkstra(const GridMap& map, const Position start,
                      const Position goal, const Connectivity connectivity) {
    if (!map.contains(start) || !map.contains(goal)) {
        throw std::out_of_range("Search endpoints must be inside GridMap");
    }
    if (map.is_blocked(start) || map.is_blocked(goal)) {
        throw std::invalid_argument("Search endpoints must be unblocked");
    }
    if (start == goal) {
        return {true, {start}, 0.0, {}};
    }

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
    // Pair ordering puts the lowest cost first, then the lowest cell index.
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
    const std::size_t start_index = index_of(start);
    distances[start_index] = 0.0;
    frontier.emplace(0.0, start_index);

    SearchResult result;
    result.stats.frontier_pushes = 1;
    while (!frontier.empty()) {
        const auto [cost, index] = frontier.top();
        frontier.pop();
        // A better route may have superseded this queued entry.
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
                frontier.emplace(next_cost, neighbor_index);
                ++result.stats.frontier_pushes;
            }
        }
    }
    return result;
}

}  // namespace dynamic_route
