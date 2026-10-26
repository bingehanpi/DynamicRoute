#include "dynamic_route/a_star.hpp"
#include "dynamic_route/bidirectional_a_star.hpp"
#include "dynamic_route/d_star_lite.hpp"
#include "dynamic_route/dijkstra.hpp"
#include "benchmark_support.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {

using namespace dynamic_route;
using namespace dynamic_route::benchmark;

constexpr int updates = 10; // Five change/restore cycles, or ten start moves.
using Clock = std::chrono::steady_clock;

enum class UpdateKind { Obstacle, TerrainWeight, StartMovement };

struct Measurements {
    double initial_us{0.0};
    double first_update_us{0.0};
    double update_us{0.0};
    SearchStats update_stats;
};

template <typename Initial, typename Update>
Measurements run_trial(const char* name, GridMap& map, const std::vector<Position>& starts,
                       const Position goal, const std::vector<double>& reference_costs,
                       Initial initial, Update update) {
    Measurements measured;
    const auto begin = Clock::now();
    SearchResult first = initial();
    const auto end = Clock::now();
    measured.initial_us = std::chrono::duration<double, std::micro>(end - begin).count();
    validate(name, map, starts.front(), goal, first, reference_costs.front());
    for (int step = 0; step < updates; ++step) {
        const auto update_begin = Clock::now();
        SearchResult result = update(step);
        const auto update_end = Clock::now();
        const double elapsed_us =
            std::chrono::duration<double, std::micro>(update_end - update_begin).count();
        measured.update_us += elapsed_us;
        if (step == 0) {
            measured.first_update_us = elapsed_us;
        }
        measured.update_stats.expanded_nodes += result.stats.expanded_nodes;
        measured.update_stats.frontier_pushes += result.stats.frontier_pushes;
        validate(name, map, starts[step + 1], goal, result, reference_costs[step + 1]);
    }
    return measured;
}

template <typename Trial>
void measure(const char* scenario, const char* name, Trial trial) {
    Measurements total;
    // Each trial creates an independent map and planner; the warm-up is discarded.
    for (int repeat = -1; repeat < repetitions; ++repeat) {
        const Measurements current = trial();
        if (repeat < 0) {
            continue;
        }
        total.initial_us += current.initial_us;
        total.first_update_us += current.first_update_us;
        total.update_us += current.update_us;
        total.update_stats.expanded_nodes += current.update_stats.expanded_nodes;
        total.update_stats.frontier_pushes += current.update_stats.frontier_pushes;
    }
    const double update_count = static_cast<double>(repetitions * updates);
    std::cout << scenario << ',' << name << ',' << total.initial_us / repetitions << ','
              << total.first_update_us / repetitions << ','
              << (total.update_us - total.first_update_us) /
                     (repetitions * (updates - 1)) << ','
              << total.update_us / update_count << ','
              << (total.initial_us + total.update_us) / repetitions << ','
              << total.update_stats.expanded_nodes / update_count << ','
              << total.update_stats.frontier_pushes / update_count << '\n';
}

void run_scenario(const char* scenario, const GridMap& base, const Position start,
                  const Position goal,
                  const std::optional<Position> off_path_cell = std::nullopt,
                  const UpdateKind update_kind = UpdateKind::Obstacle,
                  const double changed_weight = 5.0) {
    GridMap reference_map = base;
    const SearchResult original = dijkstra(reference_map, start, goal);
    if (!original.found || original.path.size() < 3) {
        throw std::runtime_error("Dynamic benchmark needs an interior path cell");
    }
    // Precompute a shared origin for every event, independent of measured planners.
    std::vector<Position> starts(updates + 1, start);
    if (update_kind == UpdateKind::StartMovement) {
        if (original.path.size() <= static_cast<std::size_t>(updates)) {
            throw std::runtime_error("Start movement needs at least ten path steps");
        }
        for (int step = 0; step < updates; ++step) {
            starts[step + 1] = original.path[step + 1];
        }
    }
    const Position changed = off_path_cell.value_or(original.path[original.path.size() / 2]);
    if (!base.contains(changed) || base.is_blocked(changed) ||
        changed == start || changed == goal) {
        throw std::runtime_error("Update cell must be free, in bounds, and not an endpoint");
    }
    if (off_path_cell &&
        std::find(original.path.begin(), original.path.end(), changed) != original.path.end()) {
        throw std::runtime_error("Off-path scenario cell lies on the original route");
    }
    const double original_weight = base.terrain_weight(changed);
    if (update_kind == UpdateKind::TerrainWeight &&
        (!std::isfinite(changed_weight) || changed_weight <= 0.0 ||
         changed_weight == original_weight)) {
        throw std::runtime_error("Changed weight must be finite, positive, and different");
    }
    // GridMap and DStarLite expose the same setters. Apply identical events to
    // the reference map, fresh-search maps, and the persistent planner.
    const auto apply_update = [&](auto& target, const bool changed_state) {
        if (update_kind == UpdateKind::TerrainWeight) {
            target.set_terrain_weight(changed, changed_state ? changed_weight : original_weight);
        } else {
            target.set_blocked(changed, changed_state);
        }
    };
    std::vector<double> reference_costs{original.total_cost};
    for (int step = 0; step < updates; ++step) {
        if (update_kind != UpdateKind::StartMovement) {
            apply_update(reference_map, step % 2 == 0);
        }
        reference_costs.push_back(dijkstra(reference_map, starts[step + 1], goal).total_cost);
    }
    std::cout << "# scenario=" << scenario << "; rows=" << base.row_count()
              << "; columns=" << base.column_count()
              << "; connectivity=Four; repetitions="
              << repetitions << "; updates=" << updates << "; warmup=1\n";
    std::cout << "# initial_start=(" << start.row << ',' << start.column
              << "); goal=(" << goal.row << ',' << goal.column << ")\n";
    if (update_kind == UpdateKind::StartMovement) {
        std::cout << "# update=start_movement; route=original_dijkstra_path; final_start=("
                  << starts.back().row << ',' << starts.back().column << ")\n";
    } else {
        std::cout << "# cell=(" << changed.row << ',' << changed.column << ")\n";
    }
    if (update_kind == UpdateKind::TerrainWeight) {
        std::cout << "# update=terrain_weight; original_weight=" << original_weight
                  << "; changed_weight=" << changed_weight << '\n';
    } else if (update_kind == UpdateKind::Obstacle) {
        std::cout << "# update=obstacle\n";
    }
    std::cout << "# initial_cost=" << reference_costs.front()
              << "; changed_cost=" << reference_costs[1]
              << "; final_cost=" << reference_costs.back() << '\n';
    std::cout << "scenario,algorithm,initial_us,first_update_us,mean_later_update_us,"
                 "mean_update_us,total_us,mean_expanded_nodes,"
                 "mean_frontier_pushes\n" << std::fixed << std::setprecision(3);

    const auto static_trial = [&](const char* name, auto planner) {
        GridMap map = base; // Map copies are outside the timed intervals.
        return run_trial(name, map, starts, goal, reference_costs,
            [&] { return planner(map, starts.front()); },
            [&](const int step) {
                if (update_kind != UpdateKind::StartMovement) {
                    apply_update(map, step % 2 == 0);
                }
                return planner(map, starts[step + 1]);
            });
    };
    measure(scenario, "Dijkstra", [&] {
        return static_trial("Dijkstra", [&](const GridMap& map, const Position current_start) {
            return dijkstra(map, current_start, goal);
        });
    });
    measure(scenario, "AStar", [&] {
        return static_trial("AStar", [&](const GridMap& map, const Position current_start) {
            return a_star(map, current_start, goal);
        });
    });
    measure(scenario, "BidirectionalAStar", [&] {
        return static_trial("BidirectionalAStar", [&](const GridMap& map, const Position current_start) {
            return bidirectional_a_star(map, current_start, goal);
        });
    });
    measure(scenario, "DStarLiteRepair", [&] {
        GridMap map = base;
        std::optional<DStarLite> planner;
        return run_trial("DStarLiteRepair", map, starts, goal, reference_costs,
            [&] {
                planner.emplace(map, start, goal); // Include construction in initial_us.
                return planner->plan();
            },
            [&](const int step) {
                if (update_kind == UpdateKind::StartMovement) {
                    planner->move_start(starts[step + 1]);
                } else {
                    apply_update(*planner, step % 2 == 0);
                }
                return planner->plan();
            });
    });
}

}  // namespace

int main() {
    try {
        // Preserve the original weighted baseline and seed without alteration.
        std::cout << "# weighted map seed=" << seed << '\n';
        const GridMap weighted = make_map();
        run_scenario("weighted_on_path", weighted, Position{0, 0},
                     Position{rows - 1, columns - 1});

        // Same open map and endpoints isolate the effect of update location.
        const GridMap open{rows, columns}; // No obstacles; every weight is 1.
        const Position start{rows / 2, 0};
        const Position goal{rows / 2, columns - 1};
        run_scenario("open_on_path", open, start, goal);
        run_scenario("open_off_path", open, start, goal, Position{0, columns / 2});
        // Raising a path cell's weight leaves the global lower bound at 1.
        run_scenario("open_terrain_on_path", open, start, goal, std::nullopt,
                     UpdateKind::TerrainWeight);
        // Only the first decrease lowers DStarLite's retained bound to 0.25.
        run_scenario("open_terrain_lower_bound", open, start, goal, std::nullopt,
                     UpdateKind::TerrainWeight, 0.25);
        run_scenario("open_moving_start", open, start, goal, std::nullopt,
                     UpdateKind::StartMovement);
    } catch (const std::exception& error) {
        std::cerr << "Dynamic benchmark failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
