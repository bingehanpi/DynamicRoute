#include "dynamic_route/bidirectional_a_star.hpp"
#include "dynamic_route/dijkstra.hpp"
#include "dynamic_route/movement.hpp"

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(BidirectionalAStarTest, RejectsOutOfBoundsEndpointsBeforeObstacles) {
    GridMap map{2, 3};
    const Position blocked{0, 0};
    map.set_blocked(blocked, true);
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {2, 0}, {0, 3}};
    for (const Position invalid : invalid_positions) {
        EXPECT_THROW(static_cast<void>(bidirectional_a_star(map, invalid, blocked)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(bidirectional_a_star(map, blocked, invalid)),
                     std::out_of_range);
    }
}

TEST(BidirectionalAStarTest, RejectsBlockedEndpointsIncludingIdenticalOnes) {
    GridMap map{1, 2};
    const Position blocked{0, 0};
    const Position free{0, 1};
    map.set_blocked(blocked, true);
    EXPECT_THROW(static_cast<void>(bidirectional_a_star(map, blocked, free)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(bidirectional_a_star(map, free, blocked)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(bidirectional_a_star(map, blocked, blocked)),
                 std::invalid_argument);
}

TEST(BidirectionalAStarTest, StartEqualToGoalNeedsNoSearchWork) {
    GridMap map{1, 1};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    const SearchResult result = bidirectional_a_star(map, Position{0, 0}, Position{0, 0});
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
    EXPECT_EQ(result.stats.expanded_nodes, 0U);
    EXPECT_EQ(result.stats.frontier_pushes, 0U);
}

TEST(BidirectionalAStarTest, ReconstructsAdjacentEndpointsWithoutDuplicatingMeeting) {
    GridMap map{1, 2};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    map.set_terrain_weight(Position{0, 1}, 0.5);
    const SearchResult result = bidirectional_a_star(map, Position{0, 0}, Position{0, 1});
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}, {0, 1}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 0.5);
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 3U);
}

TEST(BidirectionalAStarTest, BackwardSearchUsesOriginalForwardEdgeCosts) {
    GridMap map{1, 4};
    const double weights[] = {7.0, 2.0, 0.25, 0.5};
    for (int column = 0; column < 4; ++column) {
        map.set_terrain_weight(Position{0, column}, weights[column]);
    }
    const SearchResult forward = bidirectional_a_star(map, Position{0, 0}, Position{0, 3});
    const SearchResult reverse = bidirectional_a_star(map, Position{0, 3}, Position{0, 0});
    EXPECT_TRUE(forward.found);
    EXPECT_EQ(forward.path, (std::vector<Position>{{0, 0}, {0, 1}, {0, 2}, {0, 3}}));
    EXPECT_DOUBLE_EQ(forward.total_cost, 2.75);
    EXPECT_TRUE(reverse.found);
    EXPECT_EQ(reverse.path, (std::vector<Position>{{0, 3}, {0, 2}, {0, 1}, {0, 0}}));
    EXPECT_DOUBLE_EQ(reverse.total_cost, 9.25);
}

TEST(BidirectionalAStarTest, ReconstructsAnObstacleDetourAndReadsUpdatedMap) {
    GridMap map{2, 3};
    map.set_blocked(Position{0, 1}, true);
    const Position start{0, 0};
    const Position goal{0, 2};
    const SearchResult result = bidirectional_a_star(map, start, goal);
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path,
              (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 4.0);

    map.set_blocked(Position{0, 1}, false);
    map.set_terrain_weight(Position{0, 1}, 0.25);
    EXPECT_DOUBLE_EQ(bidirectional_a_star(map, start, goal).total_cost, 1.25);
}

TEST(BidirectionalAStarTest, DoesNotStopAtTheFirstDiscoveredConnection) {
    GridMap map{4, 5};
    // With Automatic, first contact costs 17; the optimal route costs 14.75.
    const double weights[][5] = {
        {0.25, 10.0, 1.0, 0.5, 1.0},
        {10.0, 1.0, 0.25, 5.0, 5.0},
        {1.0, 1.0, 5.0, 0.25, 0.5},
        {0.25, 1.0, 2.0, 0.25, 0.25}};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 5; ++column) {
            map.set_terrain_weight(Position{row, column}, weights[row][column]);
        }
    }
    map.set_blocked(Position{1, 1}, true);
    map.set_blocked(Position{2, 1}, true);
    for (const Heuristic heuristic_type :
         {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
        const SearchResult result = bidirectional_a_star(
            map, Position{0, 0}, Position{3, 4}, Connectivity::Four, heuristic_type);
        EXPECT_TRUE(result.found);
        EXPECT_DOUBLE_EQ(result.total_cost, 14.75);
        EXPECT_DOUBLE_EQ(result.total_cost,
                         dijkstra(map, Position{0, 0}, Position{3, 4}).total_cost);
    }
}

TEST(BidirectionalAStarTest, UnreachableGoalKeepsFailureOutputAndStatistics) {
    GridMap map{1, 3};
    map.set_blocked(Position{0, 1}, true);
    const SearchResult result = bidirectional_a_star(map, Position{0, 0}, Position{0, 2});
    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 2U);
}

class BidirectionalAStarConfigurationTest
    : public ::testing::TestWithParam<std::tuple<Connectivity, Heuristic>> {};

TEST_P(BidirectionalAStarConfigurationTest,
       MatchesDijkstraAndReturnsValidPathsOnSeededWeightedMaps) {
    const auto [connectivity, heuristic_type] = GetParam();
    std::mt19937 random{20261021U};
    const double weights[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
    for (int scenario = 0; scenario < 64; ++scenario) {
        SCOPED_TRACE(scenario);
        GridMap map{5, 7};
        const Position start{static_cast<int>(random() % 5U),
                             static_cast<int>(random() % 7U)};
        const Position goal{static_cast<int>(random() % 5U),
                            static_cast<int>(random() % 7U)};
        for (int row = 0; row < map.row_count(); ++row) {
            for (int column = 0; column < map.column_count(); ++column) {
                const Position position{row, column};
                const bool blocked = random() % 4U == 0U;
                map.set_blocked(position, blocked && position != start && position != goal);
                map.set_terrain_weight(position, weights[random() % 7U]);
            }
        }
        const SearchResult expected = dijkstra(map, start, goal, connectivity);
        const SearchResult result = bidirectional_a_star(
            map, start, goal, connectivity, heuristic_type);
        ASSERT_EQ(result.found, expected.found);
        if (!result.found) {
            EXPECT_TRUE(result.path.empty());
            EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
            continue;
        }
        EXPECT_NEAR(result.total_cost, expected.total_cost, 1e-10);
        ASSERT_FALSE(result.path.empty());
        EXPECT_EQ(result.path.front(), start);
        EXPECT_EQ(result.path.back(), goal);
        EXPECT_LE(result.path.size(), 35U);
        double path_cost = 0.0;
        for (std::size_t step = 1; step < result.path.size(); ++step) {
            path_cost += movement_cost(map, result.path[step - 1],
                                       result.path[step], connectivity);
        }
        EXPECT_NEAR(path_cost, result.total_cost, 1e-10);
    }
}

INSTANTIATE_TEST_SUITE_P(
    SearchOptions, BidirectionalAStarConfigurationTest,
    ::testing::Combine(::testing::Values(Connectivity::Four, Connectivity::Eight),
                       ::testing::Values(Heuristic::Automatic,
                                         Heuristic::Euclidean, Heuristic::Zero)));

TEST(BidirectionalAStarTest, EightConnectivityCombinesDiagonalAndStraightSteps) {
    const GridMap map{3, 4};
    const Position start{0, 0};
    const Position goal{2, 3};
    const SearchResult eight = bidirectional_a_star(map, start, goal, Connectivity::Eight);
    const SearchResult four = bidirectional_a_star(map, start, goal, Connectivity::Four);
    const SearchResult default_result = bidirectional_a_star(map, start, goal);

    EXPECT_TRUE(eight.found);
    ASSERT_EQ(eight.path.size(), 4U);
    EXPECT_EQ(eight.path.front(), start);
    EXPECT_EQ(eight.path.back(), goal);
    EXPECT_NEAR(eight.total_cost, 1.0 + 2.0 * std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(eight.total_cost,
                dijkstra(map, start, goal, Connectivity::Eight).total_cost, 1e-12);
    EXPECT_DOUBLE_EQ(four.total_cost, 5.0);
    EXPECT_EQ(default_result.path, four.path);
    EXPECT_DOUBLE_EQ(default_result.total_cost, four.total_cost);
    EXPECT_EQ(default_result.stats.expanded_nodes, four.stats.expanded_nodes);
    EXPECT_EQ(default_result.stats.frontier_pushes, four.stats.frontier_pushes);
}

TEST(BidirectionalAStarTest, EightConnectivityPreservesBackwardDiagonalEdgeCosts) {
    GridMap map{3, 3};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            map.set_terrain_weight(Position{row, column}, 20.0);
        }
    }
    const Position start{0, 0};
    const Position goal{2, 2};
    map.set_terrain_weight(start, 7.0);
    map.set_terrain_weight(Position{1, 1}, 2.0);
    map.set_terrain_weight(goal, 0.5);
    for (const Heuristic heuristic_type :
         {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
        const SearchResult forward = bidirectional_a_star(
            map, start, goal, Connectivity::Eight, heuristic_type);
        const SearchResult reverse = bidirectional_a_star(
            map, goal, start, Connectivity::Eight, heuristic_type);
        EXPECT_TRUE(forward.found);
        EXPECT_EQ(forward.path, (std::vector<Position>{{0, 0}, {1, 1}, {2, 2}}));
        EXPECT_NEAR(forward.total_cost, 2.5 * std::sqrt(2.0), 1e-12);
        EXPECT_TRUE(reverse.found);
        EXPECT_EQ(reverse.path, (std::vector<Position>{{2, 2}, {1, 1}, {0, 0}}));
        EXPECT_NEAR(reverse.total_cost, 9.0 * std::sqrt(2.0), 1e-12);
    }
}

TEST(BidirectionalAStarTest, EightConnectivityRespectsUpdatedObstacleCorners) {
    GridMap map{2, 2};
    const Position start{0, 0};
    const Position goal{1, 1};
    map.set_blocked(Position{0, 1}, true);
    const SearchResult detour = bidirectional_a_star(map, start, goal, Connectivity::Eight);
    EXPECT_TRUE(detour.found);
    EXPECT_EQ(detour.path, (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}}));
    EXPECT_DOUBLE_EQ(detour.total_cost, 2.0);

    map.set_blocked(Position{1, 0}, true);
    const SearchResult unreachable = bidirectional_a_star(map, start, goal, Connectivity::Eight);
    EXPECT_FALSE(unreachable.found);
    EXPECT_TRUE(unreachable.path.empty());
    EXPECT_EQ(unreachable.total_cost, std::numeric_limits<double>::infinity());

    map.set_blocked(Position{0, 1}, false);
    map.set_blocked(Position{1, 0}, false);
    EXPECT_NEAR(bidirectional_a_star(map, start, goal, Connectivity::Eight).total_cost,
                std::sqrt(2.0), 1e-12);
}

TEST(BidirectionalAStarTest, DefaultHeuristicMatchesExplicitAutomatic) {
    GridMap map{3, 4};
    map.set_terrain_weight(Position{0, 1}, 0.5);
    map.set_blocked(Position{1, 1}, true);
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        const SearchResult implicit = bidirectional_a_star(
            map, Position{0, 0}, Position{2, 3}, connectivity);
        const SearchResult explicit_choice = bidirectional_a_star(
            map, Position{0, 0}, Position{2, 3}, connectivity, Heuristic::Automatic);
        EXPECT_TRUE(implicit.found);
        EXPECT_EQ(implicit.path, explicit_choice.path);
        EXPECT_DOUBLE_EQ(implicit.total_cost, explicit_choice.total_cost);
        EXPECT_EQ(implicit.stats.expanded_nodes, explicit_choice.stats.expanded_nodes);
        EXPECT_EQ(implicit.stats.frontier_pushes, explicit_choice.stats.frontier_pushes);
    }
}

TEST(BidirectionalAStarTest, AllHeuristicsRecomputeBoundsAfterTerrainUpdates) {
    GridMap map{2, 5};
    const Position start{0, 0};
    const Position goal{0, 4};
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            EXPECT_DOUBLE_EQ(bidirectional_a_star(
                                 map, start, goal, connectivity, heuristic_type).total_cost,
                             4.0);
        }
    }
    for (int column = 0; column < map.column_count(); ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
    }
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        const SearchResult expected = dijkstra(map, start, goal, connectivity);
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            const SearchResult result = bidirectional_a_star(
                map, start, goal, connectivity, heuristic_type);
            EXPECT_TRUE(result.found);
            EXPECT_NEAR(result.total_cost, expected.total_cost, 1e-12);
            EXPECT_LT(result.total_cost, 4.0);
        }
    }
}

TEST(BidirectionalAStarTest, AllHeuristicsPreserveEndpointAndFailureBehavior) {
    GridMap map{1, 3};
    const Position start{0, 0};
    const Position blocked{0, 1};
    const Position goal{0, 2};
    map.set_blocked(blocked, true);
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            const SearchResult same = bidirectional_a_star(
                map, start, start, connectivity, heuristic_type);
            EXPECT_TRUE(same.found);
            EXPECT_EQ(same.path, (std::vector<Position>{start}));
            EXPECT_DOUBLE_EQ(same.total_cost, 0.0);
            EXPECT_EQ(same.stats.expanded_nodes, 0U);
            EXPECT_EQ(same.stats.frontier_pushes, 0U);
            EXPECT_THROW(static_cast<void>(bidirectional_a_star(
                             map, Position{-1, 0}, blocked, connectivity, heuristic_type)),
                         std::out_of_range);
            EXPECT_THROW(static_cast<void>(bidirectional_a_star(
                             map, start, blocked, connectivity, heuristic_type)),
                         std::invalid_argument);
            const SearchResult unreachable = bidirectional_a_star(
                map, start, goal, connectivity, heuristic_type);
            EXPECT_FALSE(unreachable.found);
            EXPECT_TRUE(unreachable.path.empty());
            EXPECT_EQ(unreachable.total_cost, std::numeric_limits<double>::infinity());
            EXPECT_EQ(unreachable.stats.expanded_nodes, 1U);
            EXPECT_EQ(unreachable.stats.frontier_pushes, 2U);
        }
    }
}

}  // namespace
}  // namespace dynamic_route
