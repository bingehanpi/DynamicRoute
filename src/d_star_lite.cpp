#include "dynamic_route/d_star_lite.hpp"

#include "dynamic_route/heuristic.hpp"
#include "dynamic_route/movement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dynamic_route {

DStarLite::DStarLite(GridMap& map, const Position start, const Position goal,
                     const Connectivity connectivity)
    : map_(map), start_(start), goal_(goal), connectivity_(connectivity) {
    if (!map_.contains(start_) || !map_.contains(goal_)) {
        throw std::out_of_range("Search endpoints must be inside GridMap");
    }
    if (map_.is_blocked(start_) || map_.is_blocked(goal_)) {
        throw std::invalid_argument("Search endpoints must be unblocked");
    }
}

Position DStarLite::start() const noexcept {
    return start_;
}

Position DStarLite::goal() const noexcept {
    return goal_;
}

void DStarLite::move_start(const Position position) {
    if (!map_.contains(position)) {
        throw std::out_of_range("Search start must be inside GridMap");
    }
    if (map_.is_blocked(position)) {
        throw std::invalid_argument("Search start must be unblocked");
    }
    if (position == start_) {
        return;
    }
    if (!g_.empty()) {
        // Triangle inequality keeps retained queue keys as lower bounds after
        // moving the start. Recompute each key only when that entry is popped.
        key_modifier_ += heuristic_distance(start_, position, connectivity_,
                                            Heuristic::Automatic) * minimum_weight_;
    }
    start_ = position;
}

void DStarLite::set_blocked(const Position position, const bool blocked) {
    if (!map_.contains(position)) {
        throw std::out_of_range("Obstacle position must be inside GridMap");
    }
    if (blocked && (position == start_ || position == goal_)) {
        throw std::invalid_argument("Search endpoints must remain unblocked");
    }
    if (map_.is_blocked(position) == blocked) {
        return;
    }
    if (!g_.empty()) {
        changed_cells_.push_back(position);
    }
    map_.set_blocked(position, blocked);
}

void DStarLite::set_terrain_weight(const Position position, const double weight) {
    // The getter validates bounds before weight validity, matching GridMap.
    const double previous = map_.terrain_weight(position);
    if (!std::isfinite(weight) || weight <= 0.0) {
        throw std::invalid_argument("Terrain weight must be finite and positive");
    }
    if (weight == previous) {
        return;
    }
    if (!g_.empty()) {
        changed_cells_.push_back(position);
        if (weight < minimum_weight_) {
            check_lower_bound_ = true;
        }
    }
    map_.set_terrain_weight(position, weight);
}

std::size_t DStarLite::index_of(const Position position) const noexcept {
    return static_cast<std::size_t>(position.row) *
               static_cast<std::size_t>(map_.column_count()) +
           static_cast<std::size_t>(position.column);
}

Position DStarLite::position_of(const std::size_t index) const noexcept {
    const auto columns = static_cast<std::size_t>(map_.column_count());
    return {static_cast<int>(index / columns), static_cast<int>(index % columns)};
}

DStarLite::Key DStarLite::calculate_key(const std::size_t index) const {
    const double cost = std::min(g_[index], rhs_[index]);
    const double estimate = heuristic_distance(
        start_, position_of(index), connectivity_, Heuristic::Automatic);
    return {cost + estimate * minimum_weight_ + key_modifier_, cost};
}

std::vector<Position> DStarLite::neighbors(const Position position) const {
    return connectivity_ == Connectivity::Eight
               ? eight_connected_neighbors(map_, position)
               : four_connected_neighbors(map_, position);
}

double DStarLite::minimum_terrain_weight() const {
    // Include blocked cells so a later unblock cannot invalidate the bound.
    double minimum = std::numeric_limits<double>::infinity();
    for (int row = 0; row < map_.row_count(); ++row) {
        for (int column = 0; column < map_.column_count(); ++column) {
            minimum = std::min(minimum, map_.terrain_weight(Position{row, column}));
        }
    }
    return minimum;
}

void DStarLite::rebuild_frontier(SearchStats& stats) {
    // A smaller heuristic can decrease keys: the lazy upward rekey used for
    // start movement is insufficient. Start a fresh key frame at the current
    // start, retaining all g/rhs values and pending map changes.
    key_modifier_ = 0.0;
    frontier_ = {};
    for (std::size_t index = 0; index < g_.size(); ++index) {
        if (g_[index] != rhs_[index]) {
            const auto [first, second] = calculate_key(index);
            frontier_.emplace(first, second, index, queue_versions_[index]);
            ++stats.frontier_pushes;
        }
    }
}

void DStarLite::update_vertex(const std::size_t index, SearchStats& stats) {
    const Position current = position_of(index);
    if (current != goal_) {
        rhs_[index] = std::numeric_limits<double>::infinity();
        for (const Position next : neighbors(current)) {
            rhs_[index] = std::min(rhs_[index],
                movement_cost(map_, current, next, connectivity_) + g_[index_of(next)]);
        }
    }
    // Invalidate any previous record, even if this node is now consistent.
    ++queue_versions_[index];
    if (g_[index] != rhs_[index]) {
        const auto [first, second] = calculate_key(index);
        frontier_.emplace(first, second, index, queue_versions_[index]);
        ++stats.frontier_pushes;
    }
}

void DStarLite::discard_stale_entries() {
    while (!frontier_.empty()) {
        const auto [first, second, index, version] = frontier_.top();
        // A changed start changes keys without invalidating queue records.
        // Versions distinguish replaced records from entries needing rekeying.
        if (g_[index] != rhs_[index] &&
            version == queue_versions_[index]) {
            break;
        }
        frontier_.pop();
    }
}

SearchResult DStarLite::plan() {
    if (start_ == goal_) {
        return {true, {start_}, 0.0, {}};
    }

    SearchResult result;
    const double infinity = std::numeric_limits<double>::infinity();
    const std::size_t start_index = index_of(start_);
    if (g_.empty()) {
        const auto cell_count = static_cast<std::size_t>(map_.row_count()) *
                                static_cast<std::size_t>(map_.column_count());
        g_.assign(cell_count, infinity);
        rhs_.assign(cell_count, infinity);
        queue_versions_.assign(cell_count, 0);
        minimum_weight_ = minimum_terrain_weight();
        const std::size_t goal_index = index_of(goal_);
        rhs_[goal_index] = 0.0;
        const auto [first, second] = calculate_key(goal_index);
        frontier_.emplace(first, second, goal_index, queue_versions_[goal_index]);
        ++result.stats.frontier_pushes;
    }

    if (check_lower_bound_) {
        const double minimum = minimum_terrain_weight();
        // Never raise an existing lower bound. If batched edits restored the
        // old bound, the queue can keep its current keys without rebuilding.
        if (minimum < minimum_weight_) {
            minimum_weight_ = minimum;
            rebuild_frontier(result.stats);
        }
        check_lower_bound_ = false;
    }

    // Changed edges have their tails at the changed cell or its neighbors.
    // Eight connectivity also changes diagonals between surrounding cells:
    // the changed cell can be a side cell used by the corner-cutting check.
    // Enumerate geometry rather than legal moves: a newly blocked origin has
    // no legal neighbors, but its former incoming edges still need repair.
    constexpr Position offsets[] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1},
                                    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    for (const Position changed : changed_cells_) {
        update_vertex(index_of(changed), result.stats);
        for (const Position offset : offsets) {
            if (connectivity_ == Connectivity::Four &&
                offset.row != 0 && offset.column != 0) {
                continue;
            }
            const Position adjacent{changed.row + offset.row,
                                    changed.column + offset.column};
            if (map_.contains(adjacent)) {
                update_vertex(index_of(adjacent), result.stats);
            }
        }
    }
    changed_cells_.clear();

    while (true) {
        discard_stale_entries();
        if (frontier_.empty()) {
            break;
        }
        const auto [first, second, index, version] = frontier_.top();
        const double start_priority = calculate_key(start_index).first;
        // Diagonal costs and accumulated km can round equal f-values apart.
        // Conservatively finish the narrow f-value band around the start;
        // otherwise a stale, too-cheap g downstream can survive termination.
        // Heap ordering stays exact to preserve a strict weak ordering.
        const double rounding_band = 32.0 * std::numeric_limits<double>::epsilon() *
                                     std::max(1.0, std::abs(start_priority));
        if (first > start_priority + rounding_band &&
            rhs_[start_index] == g_[start_index]) {
            break;
        }
        frontier_.pop();
        const Key current_key = calculate_key(index);
        if (Key{first, second} < current_key) {
            // This record is current, but its priority predates a start move.
            frontier_.emplace(current_key.first, current_key.second, index, version);
            ++result.stats.frontier_pushes;
            continue;
        }
        const Position current = position_of(index);
        ++result.stats.expanded_nodes;
        if (g_[index] > rhs_[index]) {
            g_[index] = rhs_[index];
        } else {
            g_[index] = infinity;
            update_vertex(index, result.stats);
        }
        // Adjacency is symmetric, but costs are not: update_vertex(predecessor)
        // evaluates predecessor -> current, charging current's terrain weight.
        for (const Position predecessor : neighbors(current)) {
            update_vertex(index_of(predecessor), result.stats);
        }
    }

    if (!std::isfinite(g_[start_index])) {
        return result;
    }

    // Follow the successor minimizing c(current, successor) + g(successor).
    // Sum forward costs to keep total_cost consistent with the returned path.
    Position current = start_;
    result.path.push_back(current);
    double path_cost = 0.0;
    while (current != goal_) {
        Position best = current;
        double best_cost = infinity;
        for (const Position next : neighbors(current)) {
            const double cost = movement_cost(map_, current, next, connectivity_) +
                                g_[index_of(next)];
            if (cost < best_cost) {
                best_cost = cost;
                best = next;
            }
        }
        if (best == current || result.path.size() >= g_.size()) {
            throw std::logic_error("DStarLite path reconstruction did not reach goal");
        }
        path_cost += movement_cost(map_, current, best, connectivity_);
        current = best;
        result.path.push_back(current);
    }
    result.found = true;
    result.total_cost = path_cost;
    return result;
}

}  // namespace dynamic_route
