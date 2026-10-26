#pragma once

#include "dynamic_route/grid_map.hpp"
#include "dynamic_route/movement.hpp"
#include "dynamic_route/search_result.hpp"

#include <functional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace dynamic_route {

// The map is borrowed and must outlive this object. Do not replace, move, or
// modify it directly while the planner is in use. Obstacle and terrain updates
// go through this planner so its persistent search state can be repaired. Do not share a
// changing map between multiple planners: each needs its own map and updates.
// Supports Four/Eight connectivity and finite positive terrain weights.
// Diagonal movement uses the movement layer's costs and corner checks.
class DStarLite {
public:
    // Checks both endpoints' bounds before obstacle state, matching dijkstra.
    // Throws std::out_of_range for positions outside the map and
    // std::invalid_argument for blocked endpoints, even when start == goal.
    // Construction validates endpoints only; it does not establish reachability.
    DStarLite(GridMap& map, Position start, Position goal,
              Connectivity connectivity = Connectivity::Four);

    [[nodiscard]] Position start() const noexcept;
    [[nodiscard]] Position goal() const noexcept;

    // Sets the current position; adjacency to the old start is not required.
    // Throws std::out_of_range outside the map or std::invalid_argument on an
    // obstacle. Rejected moves and moving to the same start do nothing.
    // Retains search state; plan() computes the route from this new start.
    void move_start(Position position);

    // Updates the borrowed map immediately; plan() performs the search repair.
    // Throws std::out_of_range for an invalid position and std::invalid_argument
    // when blocking either endpoint. Rejected and unchanged updates do nothing.
    // Multiple updates may be made before plan().
    void set_blocked(Position position, bool blocked);

    // Updates the borrowed map immediately; plan() repairs affected edge costs.
    // Matches GridMap validation: bounds first, then finite positive weight.
    // Rejected and unchanged updates do nothing. Endpoints and blocked cells
    // may have their weights changed without changing their obstacle state.
    // A new lower heuristic bound requires rekeying the retained frontier.
    void set_terrain_weight(Position position, double weight);

    // Finds a minimum-cost path, retaining search state for subsequent calls.
    // Failure and start == goal follow SearchResult's existing conventions.
    // Statistics count only this call's search work, excluding path extraction.
    // Includes processing recorded map updates and any required rekeying. Calls without
    // changes perform no further search work.
    [[nodiscard]] SearchResult plan();

private:
    using Key = std::pair<double, double>;
    // Lexicographic key, then cell index for deterministic ties.
    // Last field identifies the current queue record for that cell.
    using Entry = std::tuple<double, double, std::size_t, std::size_t>;

    [[nodiscard]] std::size_t index_of(Position position) const noexcept;
    [[nodiscard]] Position position_of(std::size_t index) const noexcept;
    [[nodiscard]] Key calculate_key(std::size_t index) const;
    [[nodiscard]] std::vector<Position> neighbors(Position position) const;
    [[nodiscard]] double minimum_terrain_weight() const;
    void rebuild_frontier(SearchStats& stats);
    void update_vertex(std::size_t index, SearchStats& stats);
    void discard_stale_entries();

    GridMap& map_;
    Position start_;
    Position goal_;
    Connectivity connectivity_;
    // Empty until the first nontrivial plan(). All costs point toward the goal.
    std::vector<double> g_;
    std::vector<double> rhs_;
    std::vector<std::size_t> queue_versions_;
    std::vector<Position> changed_cells_;
    double minimum_weight_{0.0};
    double key_modifier_{0.0};
    bool check_lower_bound_{false};
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier_;
};

}  // namespace dynamic_route
