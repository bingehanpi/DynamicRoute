#include "dynamic_route/a_star.hpp"
#include "dynamic_route/bidirectional_a_star.hpp"
#include "dynamic_route/d_star_lite.hpp"
#include "dynamic_route/dijkstra.hpp"
#include "benchmark_support.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

using namespace dynamic_route;
using namespace dynamic_route::benchmark;

template <typename Planner>
void measure(const char* name, const GridMap& map, const Position start,
             const Position goal, const double reference_cost, Planner planner) {
    // One unmeasured warm-up. Map creation, validation, and output are not timed.
    validate(name, map, start, goal, planner(), reference_cost);
    double total_microseconds = 0.0;
    SearchResult result;
    for (int repeat = 0; repeat < repetitions; ++repeat) {
        const auto begin = std::chrono::steady_clock::now();
        SearchResult current = planner();
        const auto end = std::chrono::steady_clock::now();
        total_microseconds +=
            std::chrono::duration<double, std::micro>(end - begin).count();
        validate(name, map, start, goal, current, reference_cost);
        result = std::move(current);
    }
    // Identical deterministic inputs give identical costs and search counters.
    std::cout << name << ',' << result.total_cost << ',' << result.stats.expanded_nodes
              << ',' << result.stats.frontier_pushes << ','
              << total_microseconds / repetitions << '\n';
}

}  // namespace

int main() {
    try {
        GridMap map = make_map();
        const Position start{0, 0};
        const Position goal{rows - 1, columns - 1};
        const SearchResult reference = dijkstra(map, start, goal);
        if (!reference.found || !std::isfinite(reference.total_cost)) {
            throw std::runtime_error("Benchmark corridor must provide a finite route");
        }
        std::cout << "# static initial planning; rows=" << rows << "; columns=" << columns
                  << "; seed=" << seed << "; connectivity=Four; repetitions="
                  << repetitions << "; warmup=1\n";
        std::cout << "algorithm,total_cost,expanded_nodes,frontier_pushes,mean_us\n"
                  << std::fixed << std::setprecision(3);
        measure("Dijkstra", map, start, goal, reference.total_cost,
                [&] { return dijkstra(map, start, goal); });
        measure("AStar", map, start, goal, reference.total_cost,
                [&] { return a_star(map, start, goal); });
        measure("BidirectionalAStar", map, start, goal, reference.total_cost,
                [&] { return bidirectional_a_star(map, start, goal); });
        measure("DStarLiteInitial", map, start, goal, reference.total_cost, [&] {
            // Construct and destroy a fresh planner inside every timed call,
            // including its initialization and first search, as for the others.
            DStarLite planner{map, start, goal};
            return planner.plan();
        });
    } catch (const std::exception& error) {
        std::cerr << "Benchmark failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
