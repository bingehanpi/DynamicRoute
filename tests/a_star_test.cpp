#include "dynamic_route/a_star.hpp"
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

TEST(AStarTest, RejectsOutOfBoundsEndpointsBeforeCheckingObstacles) {
    GridMap map{2, 3};
    const Position blocked{0, 0};
    map.set_blocked(blocked, true);
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {2, 0}, {0, 3}};

    for (const Position invalid : invalid_positions) {
        EXPECT_THROW(static_cast<void>(a_star(map, invalid, blocked)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(a_star(map, blocked, invalid)),
                     std::out_of_range);
    }
}

TEST(AStarTest, RejectsBlockedEndpointsIncludingStartEqualToGoal) {
    GridMap map{1, 2};
    const Position blocked{0, 0};
    const Position free{0, 1};
    map.set_blocked(blocked, true);

    EXPECT_THROW(static_cast<void>(a_star(map, blocked, free)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(a_star(map, free, blocked)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(a_star(map, blocked, blocked)),
                 std::invalid_argument);
}

TEST(AStarTest, StartEqualToGoalNeedsNoMovesOrSearchWork) {
    GridMap map{1, 1};
    const Position start{0, 0};
    map.set_terrain_weight(start, 9.0);

    const SearchResult result = a_star(map, start, start);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{start}));
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
    EXPECT_EQ(result.stats.expanded_nodes, 0U);
    EXPECT_EQ(result.stats.frontier_pushes, 0U);
}

TEST(AStarTest, ChargesOnlyTheDestinationForASingleMove) {
    GridMap map{1, 2};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    map.set_terrain_weight(Position{0, 1}, 2.5);

    const SearchResult result = a_star(map, Position{0, 0}, Position{0, 1});

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}, {0, 1}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 2.5);
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 2U);
    EXPECT_DOUBLE_EQ(a_star(map, Position{0, 1}, Position{0, 0}).total_cost, 7.0);
}

TEST(AStarTest, ReconstructsADetourAroundAnObstacle) {
    GridMap map{2, 3};
    map.set_blocked(Position{0, 1}, true);
    const std::vector<Position> expected{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}};

    const SearchResult result = a_star(map, Position{0, 0}, Position{0, 2});

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, expected);
    EXPECT_DOUBLE_EQ(result.total_cost,
                     dijkstra(map, Position{0, 0}, Position{0, 2}).total_cost);
}

TEST(AStarTest, FindsACheaperDetourWithWeightsBelowOne) {
    GridMap map{2, 5};
    for (int column = 0; column < map.column_count(); ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
    }
    const Position start{0, 0};
    const Position goal{0, 4};
    const std::vector<Position> expected{
        {0, 0}, {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {0, 4}};

    const SearchResult result = a_star(map, start, goal);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, expected);
    EXPECT_NEAR(result.total_cost, 1.5, 1e-12);
    EXPECT_DOUBLE_EQ(result.total_cost, dijkstra(map, start, goal).total_cost);
}

TEST(AStarTest, UnreachableGoalKeepsFailureOutputAndRecordsSearchWork) {
    GridMap map{1, 3};
    map.set_blocked(Position{0, 1}, true);

    const SearchResult result = a_star(map, Position{0, 0}, Position{0, 2});

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(result.stats.expanded_nodes, 1U);
    EXPECT_EQ(result.stats.frontier_pushes, 1U);
}

TEST(AStarTest, SeparateSearchesReadUpdatedObstaclesAndTerrainWeights) {
    GridMap map{2, 5};
    const Position start{0, 0};
    const Position goal{0, 4};
    EXPECT_DOUBLE_EQ(a_star(map, start, goal).total_cost, 4.0);

    for (int column = 0; column < map.column_count(); ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
    }
    EXPECT_NEAR(a_star(map, start, goal).total_cost, 1.5, 1e-12);

    map.set_blocked(Position{1, 2}, true);
    const SearchResult result = a_star(map, start, goal);
    EXPECT_TRUE(result.found);
    EXPECT_DOUBLE_EQ(result.total_cost, dijkstra(map, start, goal).total_cost);
    EXPECT_TRUE(map.is_blocked(Position{1, 2}));
    EXPECT_DOUBLE_EQ(map.terrain_weight(Position{1, 2}), 0.1);
}

class AStarConfigurationTest
    : public ::testing::TestWithParam<std::tuple<Connectivity, Heuristic>> {};

TEST_P(AStarConfigurationTest, MatchesDijkstraAndReturnsValidPathsOnSeededWeightedMaps) {
    const auto [connectivity, heuristic_type] = GetParam();
    std::mt19937 random{20261020U};
    const double weights[] = {0.25, 0.5, 1.0, 2.0, 5.0};
    const Position start{0, 0};
    const Position goal{4, 6};
    for (int scenario = 0; scenario < 24; ++scenario) {
        SCOPED_TRACE(scenario);
        GridMap map{5, 7};
        for (int row = 0; row < map.row_count(); ++row) {
            for (int column = 0; column < map.column_count(); ++column) {
                const Position position{row, column};
                const bool blocked = random() % 5U == 0U;
                map.set_blocked(position, blocked && position != start &&
                                             position != goal);
                map.set_terrain_weight(position, weights[random() % 5U]);
            }
        }
        const SearchResult expected = dijkstra(map, start, goal, connectivity);
        const SearchResult result = a_star(map, start, goal, connectivity,
                                           heuristic_type);
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
        double path_cost = 0.0;
        for (std::size_t step = 1; step < result.path.size(); ++step) {
            path_cost += movement_cost(map, result.path[step - 1],
                                       result.path[step], connectivity);
        }
        EXPECT_DOUBLE_EQ(path_cost, result.total_cost);
    }
}

INSTANTIATE_TEST_SUITE_P(
    SearchOptions, AStarConfigurationTest,
    ::testing::Combine(::testing::Values(Connectivity::Four, Connectivity::Eight),
                       ::testing::Values(Heuristic::Automatic,
                                         Heuristic::Euclidean, Heuristic::Zero)));

TEST(AStarTest, ExpandsFewerNodesOnAnOpenMapWithAStraightRoute) {
    const GridMap map{5, 5};
    const Position start{2, 0};
    const Position goal{2, 4};
    const SearchResult reference = dijkstra(map, start, goal);
    const SearchResult result = a_star(map, start, goal);

    EXPECT_TRUE(result.found);
    EXPECT_DOUBLE_EQ(result.total_cost, reference.total_cost);
    EXPECT_LT(result.stats.expanded_nodes, reference.stats.expanded_nodes);
}

TEST(AStarTest, EightConnectivityCombinesDiagonalAndStraightSteps) {
    const GridMap map{3, 4};
    const Position start{0, 0};
    const Position goal{2, 3};
    const SearchResult result = a_star(map, start, goal, Connectivity::Eight);
    const SearchResult four = a_star(map, start, goal, Connectivity::Four);

    EXPECT_TRUE(result.found);
    ASSERT_EQ(result.path.size(), 4U);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), goal);
    EXPECT_NEAR(result.total_cost, 1.0 + 2.0 * std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(result.total_cost,
                dijkstra(map, start, goal, Connectivity::Eight).total_cost, 1e-12);
    EXPECT_DOUBLE_EQ(four.total_cost, 5.0);
    EXPECT_EQ(a_star(map, start, goal).path, four.path);
}

TEST(AStarTest, EightConnectivityDetoursOrFailsInsteadOfCuttingCorners) {
    GridMap map{2, 2};
    const Position start{0, 0};
    const Position goal{1, 1};
    map.set_blocked(Position{0, 1}, true);
    const SearchResult detour = a_star(map, start, goal, Connectivity::Eight);
    EXPECT_TRUE(detour.found);
    EXPECT_EQ(detour.path, (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}}));
    EXPECT_DOUBLE_EQ(detour.total_cost, 2.0);

    map.set_blocked(Position{1, 0}, true);
    const SearchResult unreachable = a_star(map, start, goal, Connectivity::Eight);
    EXPECT_FALSE(unreachable.found);
    EXPECT_TRUE(unreachable.path.empty());
    EXPECT_EQ(unreachable.total_cost, std::numeric_limits<double>::infinity());
}

TEST(AStarTest, EightConnectivityRecomputesHeuristicForLowerTerrainWeights) {
    GridMap map{2, 5};
    const Position start{0, 0};
    const Position goal{0, 4};
    EXPECT_DOUBLE_EQ(a_star(map, start, goal, Connectivity::Eight).total_cost, 4.0);
    for (int column = 0; column < map.column_count(); ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
    }

    const SearchResult result = a_star(map, start, goal, Connectivity::Eight);
    EXPECT_TRUE(result.found);
    EXPECT_NEAR(result.total_cost, 1.3 + 0.1 * std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(result.total_cost,
                dijkstra(map, start, goal, Connectivity::Eight).total_cost, 1e-12);
}

TEST(AStarTest, DefaultHeuristicMatchesExplicitAutomaticChoice) {
    GridMap map{3, 4};
    map.set_terrain_weight(Position{0, 1}, 0.5);
    map.set_blocked(Position{1, 1}, true);
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        const SearchResult implicit = a_star(map, Position{0, 0}, Position{2, 3},
                                             connectivity);
        const SearchResult explicit_choice = a_star(
            map, Position{0, 0}, Position{2, 3}, connectivity, Heuristic::Automatic);
        EXPECT_TRUE(implicit.found);
        EXPECT_EQ(implicit.path, explicit_choice.path);
        EXPECT_DOUBLE_EQ(implicit.total_cost, explicit_choice.total_cost);
        EXPECT_EQ(implicit.stats.expanded_nodes, explicit_choice.stats.expanded_nodes);
        EXPECT_EQ(implicit.stats.frontier_pushes, explicit_choice.stats.frontier_pushes);
    }
}

TEST(AStarTest, ZeroHeuristicMatchesDijkstraPathCostAndSearchStatistics) {
    GridMap map{3, 4};
    const Position start{0, 0};
    const Position goal{2, 3};
    map.set_terrain_weight(Position{0, 1}, 0.25);
    map.set_terrain_weight(Position{1, 2}, 5.0);
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        const SearchResult expected = dijkstra(map, start, goal, connectivity);
        const SearchResult result = a_star(map, start, goal, connectivity, Heuristic::Zero);
        EXPECT_TRUE(result.found);
        EXPECT_EQ(result.path, expected.path);
        EXPECT_DOUBLE_EQ(result.total_cost, expected.total_cost);
        EXPECT_EQ(result.stats.expanded_nodes, expected.stats.expanded_nodes);
        EXPECT_EQ(result.stats.frontier_pushes, expected.stats.frontier_pushes);
    }
}

TEST(AStarTest, AllHeuristicsFindCheaperTerrainDetoursInBothConnectivityModes) {
    GridMap map{2, 5};
    const Position start{0, 0};
    const Position goal{0, 4};
    for (int column = 0; column < map.column_count(); ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
    }
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        const SearchResult expected = dijkstra(map, start, goal, connectivity);
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            const SearchResult result = a_star(map, start, goal, connectivity,
                                               heuristic_type);
            EXPECT_TRUE(result.found);
            EXPECT_NEAR(result.total_cost, expected.total_cost, 1e-12);
            EXPECT_LT(result.total_cost, 4.0);
        }
    }
}

TEST(AStarTest, AllHeuristicsPreserveEndpointAndUnreachableGoalBehavior) {
    GridMap map{1, 3};
    const Position start{0, 0};
    const Position blocked{0, 1};
    const Position goal{0, 2};
    map.set_blocked(blocked, true);
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            const SearchResult same = a_star(map, start, start, connectivity, heuristic_type);
            EXPECT_TRUE(same.found);
            EXPECT_EQ(same.path, (std::vector<Position>{start}));
            EXPECT_DOUBLE_EQ(same.total_cost, 0.0);
            EXPECT_EQ(same.stats.expanded_nodes, 0U);
            EXPECT_EQ(same.stats.frontier_pushes, 0U);
            EXPECT_THROW(static_cast<void>(a_star(
                             map, Position{-1, 0}, blocked, connectivity, heuristic_type)),
                         std::out_of_range);
            EXPECT_THROW(static_cast<void>(a_star(
                             map, start, blocked, connectivity, heuristic_type)),
                         std::invalid_argument);
            const SearchResult result = a_star(map, start, goal, connectivity, heuristic_type);
            EXPECT_FALSE(result.found);
            EXPECT_TRUE(result.path.empty());
            EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
            EXPECT_EQ(result.stats.expanded_nodes, 1U);
            EXPECT_EQ(result.stats.frontier_pushes, 1U);
        }
    }
}

}  // namespace
}  // namespace dynamic_route
