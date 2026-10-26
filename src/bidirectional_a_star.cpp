#include "dynamic_route/bidirectional_a_star.hpp"

#include "dynamic_route/movement.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace dynamic_route {
namespace {

// Each direction owns its search state; entries contain f, g, and cell index.
using Entry = std::tuple<double, double, std::size_t>;
struct SearchSide {
    explicit SearchSide(const std::size_t cell_count)
        : distances(cell_count, std::numeric_limits<double>::infinity()),
          parents(cell_count) {}

    std::vector<double> distances;
    std::vector<std::size_t> parents;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
};

}  // namespace

SearchResult bidirectional_a_star(const GridMap& map, const Position start,
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
    const auto heuristic = [minimum_weight, connectivity, heuristic_type](
                               const Position from, const Position to) {
        return heuristic_distance(from, to, connectivity, heuristic_type) *
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
    const auto discard_stale = [](SearchSide& side) {
        while (!side.frontier.empty()) {
            const auto [estimate, cost, index] = side.frontier.top();
            if (cost == side.distances[index]) {
                break;
            }
            side.frontier.pop();
        }
    };

    SearchSide forward{cell_count};
    SearchSide backward{cell_count};
    const std::size_t start_index = index_of(start);
    const std::size_t goal_index = index_of(goal);
    forward.distances[start_index] = 0.0;
    backward.distances[goal_index] = 0.0;
    forward.frontier.emplace(heuristic(start, goal), 0.0, start_index);
    backward.frontier.emplace(heuristic(goal, start), 0.0, goal_index);

    SearchResult result;
    result.stats.frontier_pushes = 2;
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t meeting = cell_count;
    bool forward_turn = true;
    while (true) {
        discard_stale(forward);
        discard_stale(backward);
        if (forward.frontier.empty() || backward.frontier.empty()) {
            break;
        }
        // First contact only gives an upper bound. Either direction's minimum
        // f can certify that no cheaper complete path remains to be found.
        if (std::get<0>(forward.frontier.top()) >= best_cost ||
            std::get<0>(backward.frontier.top()) >= best_cost) {
            break;
        }

        SearchSide& active = forward_turn ? forward : backward;
        const SearchSide& other = forward_turn ? backward : forward;
        const Position target = forward_turn ? goal : start;
        const auto [estimate, cost, index] = active.frontier.top();
        active.frontier.pop();
        const Position current = position_of(index);
        ++result.stats.expanded_nodes;
        const auto neighbors = connectivity == Connectivity::Eight
                                   ? eight_connected_neighbors(map, current)
                                   : four_connected_neighbors(map, current);
        for (const Position neighbor : neighbors) {
            const std::size_t neighbor_index = index_of(neighbor);
            // In the reversed graph, current -> neighbor represents the
            // original edge neighbor -> current, whose destination is current.
            const double step_cost =
                forward_turn ? movement_cost(map, current, neighbor, connectivity)
                             : movement_cost(map, neighbor, current, connectivity);
            const double next_cost = cost + step_cost;
            if (next_cost < active.distances[neighbor_index]) {
                active.distances[neighbor_index] = next_cost;
                active.parents[neighbor_index] = index;
                active.frontier.emplace(next_cost + heuristic(neighbor, target),
                                        next_cost, neighbor_index);
                ++result.stats.frontier_pushes;
                const double joined_cost = next_cost + other.distances[neighbor_index];
                if (joined_cost < best_cost) {
                    best_cost = joined_cost;
                    meeting = neighbor_index;
                }
            }
        }
        forward_turn = !forward_turn;
    }

    if (meeting == cell_count) {
        return result;
    }
    result.found = true;
    result.total_cost = best_cost;
    for (std::size_t step = meeting;; step = forward.parents[step]) {
        result.path.push_back(position_of(step));
        if (step == start_index) {
            break;
        }
    }
    std::reverse(result.path.begin(), result.path.end());
    for (std::size_t step = meeting; step != goal_index;) {
        step = backward.parents[step];
        result.path.push_back(position_of(step));
    }
    return result;
}

}  // namespace dynamic_route
