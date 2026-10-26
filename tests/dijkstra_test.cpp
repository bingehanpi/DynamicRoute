#include "dynamic_route/dijkstra.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(DijkstraTest, RejectsOutOfBoundsEndpointsBeforeCheckingObstacles) {
    GridMap map{2, 3};
    const Position blocked{0, 0};
    map.set_blocked(blocked, true);
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {2, 0}, {0, 3}};

    for (const Position invalid : invalid_positions) {
        EXPECT_THROW(static_cast<void>(dijkstra(map, invalid, blocked)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(dijkstra(map, blocked, invalid)),
                     std::out_of_range);
    }
}

TEST(DijkstraTest, RejectsBlockedEndpointsIncludingStartEqualToGoal) {
    GridMap map{1, 2};
    const Position blocked{0, 0};
    const Position free{0, 1};
    map.set_blocked(blocked, true);

    EXPECT_THROW(static_cast<void>(dijkstra(map, blocked, free)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(dijkstra(map, free, blocked)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(dijkstra(map, blocked, blocked)),
                 std::invalid_argument);
}

TEST(DijkstraTest, StartEqualToGoalNeedsNoMovesOrSearchWork) {
    GridMap map{1, 1};
    const Position start{0, 0};
    map.set_terrain_weight(start, 9.0);

    const SearchResult result = dijkstra(map, start, start);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{start}));
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
    EXPECT_EQ(result.stats.expanded_nodes, 0U);
    EXPECT_EQ(result.stats.frontier_pushes, 0U);
}

TEST(DijkstraTest, ChargesOnlyTheDestinationForASingleMove) {
    GridMap map{1, 2};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    map.set_terrain_weight(Position{0, 1}, 2.5);

    const SearchResult result = dijkstra(map, Position{0, 0}, Position{0, 1});

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}, {0, 1}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 2.5);
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 2U);
    EXPECT_DOUBLE_EQ(dijkstra(map, Position{0, 1}, Position{0, 0}).total_cost, 7.0);
}

TEST(DijkstraTest, ReconstructsADetourAroundAnObstacle) {
    GridMap map{2, 3};
    map.set_blocked(Position{0, 1}, true);
    const std::vector<Position> expected{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}};

    const SearchResult result = dijkstra(map, Position{0, 0}, Position{0, 2});

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, expected);
    EXPECT_DOUBLE_EQ(result.total_cost, 4.0);
}

TEST(DijkstraTest, PrefersALongerPathWhenItsTotalCostIsLower) {
    GridMap map{2, 3};
    map.set_terrain_weight(Position{0, 1}, 10.0);
    map.set_terrain_weight(Position{1, 1}, 0.5);
    const std::vector<Position> expected{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}};

    const SearchResult result = dijkstra(map, Position{0, 0}, Position{0, 2});

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, expected);
    EXPECT_DOUBLE_EQ(result.total_cost, 3.5);
}

TEST(DijkstraTest, UnreachableGoalKeepsFailureOutputAndRecordsSearchWork) {
    GridMap map{1, 3};
    map.set_blocked(Position{0, 1}, true);

    const SearchResult result = dijkstra(map, Position{0, 0}, Position{0, 2});

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 1U);
}

TEST(DijkstraTest, SeparateSearchesReadCurrentMapAndHaveIndependentState) {
    GridMap map{1, 3};
    const Position start{0, 0};
    const Position goal{0, 2};
    map.set_blocked(Position{0, 1}, true);
    EXPECT_FALSE(dijkstra(map, start, goal).found);

    map.set_blocked(Position{0, 1}, false);
    map.set_terrain_weight(Position{0, 1}, 2.5);
    const SearchResult result = dijkstra(map, start, goal);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}, {0, 1}, {0, 2}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 3.5);
    EXPECT_EQ(result.stats.expanded_nodes, 2U);
    EXPECT_EQ(result.stats.frontier_pushes, 3U);
    EXPECT_DOUBLE_EQ(map.terrain_weight(Position{0, 1}), 2.5);
    EXPECT_FALSE(map.is_blocked(Position{0, 1}));
}

TEST(DijkstraTest, EightConnectivityCombinesDiagonalAndStraightSteps) {
    const GridMap map{3, 4};
    const Position start{0, 0};
    const Position goal{2, 3};
    const SearchResult result = dijkstra(map, start, goal, Connectivity::Eight);
    const SearchResult four = dijkstra(map, start, goal, Connectivity::Four);
    const SearchResult default_result = dijkstra(map, start, goal);

    EXPECT_TRUE(result.found);
    ASSERT_EQ(result.path.size(), 4U);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), goal);
    EXPECT_NEAR(result.total_cost, 1.0 + 2.0 * std::sqrt(2.0), 1e-12);
    EXPECT_DOUBLE_EQ(four.total_cost, 5.0);
    EXPECT_EQ(default_result.path, four.path);
    EXPECT_DOUBLE_EQ(default_result.total_cost, four.total_cost);
}

TEST(DijkstraTest, EightConnectivityChargesDestinationWeightForDiagonals) {
    GridMap map{2, 2};
    const Position start{0, 0};
    const Position goal{1, 1};
    map.set_terrain_weight(start, 7.0);
    map.set_terrain_weight(goal, 0.5);
    map.set_terrain_weight(Position{0, 1}, 5.0);
    map.set_terrain_weight(Position{1, 0}, 5.0);

    const SearchResult forward = dijkstra(map, start, goal, Connectivity::Eight);
    const SearchResult reverse = dijkstra(map, goal, start, Connectivity::Eight);

    EXPECT_TRUE(forward.found);
    EXPECT_EQ(forward.path, (std::vector<Position>{start, goal}));
    EXPECT_DOUBLE_EQ(forward.total_cost, std::sqrt(2.0) * 0.5);
    EXPECT_TRUE(reverse.found);
    EXPECT_EQ(reverse.path, (std::vector<Position>{goal, start}));
    EXPECT_DOUBLE_EQ(reverse.total_cost, std::sqrt(2.0) * 7.0);
}

TEST(DijkstraTest, EightConnectivityDetoursOrFailsInsteadOfCuttingCorners) {
    GridMap map{2, 2};
    const Position start{0, 0};
    const Position goal{1, 1};
    map.set_blocked(Position{0, 1}, true);
    const SearchResult detour = dijkstra(map, start, goal, Connectivity::Eight);

    EXPECT_TRUE(detour.found);
    EXPECT_EQ(detour.path, (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}}));
    EXPECT_DOUBLE_EQ(detour.total_cost, 2.0);

    map.set_blocked(Position{1, 0}, true);
    const SearchResult unreachable = dijkstra(map, start, goal, Connectivity::Eight);
    EXPECT_FALSE(unreachable.found);
    EXPECT_TRUE(unreachable.path.empty());
    EXPECT_EQ(unreachable.total_cost, std::numeric_limits<double>::infinity());
}

}  // namespace
}  // namespace dynamic_route
