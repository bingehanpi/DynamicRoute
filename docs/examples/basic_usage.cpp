#include "dynamic_route/a_star.hpp"
#include "dynamic_route/bidirectional_a_star.hpp"
#include "dynamic_route/d_star_lite.hpp"
#include "dynamic_route/dijkstra.hpp"

#include <exception>
#include <iomanip>
#include <iostream>

namespace {

void print_result(const char* label, const dynamic_route::SearchResult& result) {
    std::cout << label << ": found=" << std::boolalpha << result.found
              << ", cost=" << result.total_cost
              << ", expanded=" << result.stats.expanded_nodes
              << ", pushes=" << result.stats.frontier_pushes << '\n';
    std::cout << "  path:";
    for (const auto position : result.path) {
        std::cout << " (" << position.row << ',' << position.column << ')';
    }
    std::cout << '\n';
}

}  // namespace

int main() {
    try {
        using namespace dynamic_route;
        GridMap map{5, 7}; // Initially free, with terrain weight 1 everywhere.
        const Position start{2, 0};
        const Position goal{2, 6};
        std::cout << std::fixed << std::setprecision(2);

        print_result("Dijkstra", dijkstra(map, start, goal));
        print_result("AStar", a_star(map, start, goal));
        print_result("BidirectionalAStar", bidirectional_a_star(map, start, goal));

        // All map changes below go through the planner that borrows this map.
        DStarLite planner{map, start, goal};
        print_result("DStarLite initial", planner.plan());
        print_result("DStarLite unchanged", planner.plan());

        planner.set_blocked(Position{2, 3}, true);
        print_result("DStarLite blocked", planner.plan());
        planner.set_blocked(Position{2, 3}, false);
        print_result("DStarLite restored", planner.plan());

        planner.set_terrain_weight(Position{2, 3}, 0.5);
        print_result("DStarLite cheaper terrain", planner.plan());
        planner.move_start(Position{2, 1});
        print_result("DStarLite moved start", planner.plan());
    } catch (const std::exception& error) {
        std::cerr << "Example failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
