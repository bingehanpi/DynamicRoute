#include "dynamic_route/d_star_lite.hpp"
#include "dynamic_route/dijkstra.hpp"
#include "dynamic_route/movement.hpp"

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

void expect_matches_dijkstra(GridMap& map, DStarLite& planner,
                              const Connectivity connectivity = Connectivity::Four) {
    const SearchResult expected = dijkstra(map, planner.start(), planner.goal(), connectivity);
    SearchResult result;
    ASSERT_NO_THROW(result = planner.plan());
    ASSERT_EQ(result.found, expected.found);
    if (!result.found) {
        EXPECT_TRUE(result.path.empty());
        EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
        return;
    }
    EXPECT_NEAR(result.total_cost, expected.total_cost, 1e-10);
    ASSERT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.front(), planner.start());
    EXPECT_EQ(result.path.back(), planner.goal());
    ASSERT_LE(result.path.size(),
              static_cast<std::size_t>(map.row_count() * map.column_count()));
    double cost = 0.0;
    for (std::size_t step = 1; step < result.path.size(); ++step) {
        cost += movement_cost(map, result.path[step - 1], result.path[step], connectivity);
    }
    EXPECT_DOUBLE_EQ(cost, result.total_cost);
}

TEST(DStarLiteTest, StoresValidatedEndpoints) {
    GridMap map{2, 3};
    const Position start{1, 0};
    const Position goal{0, 2};

    const DStarLite planner{map, start, goal};

    EXPECT_EQ(planner.start(), start);
    EXPECT_EQ(planner.goal(), goal);
}

TEST(DStarLiteTest, RejectsOutOfBoundsEndpointsBeforeCheckingObstacles) {
    GridMap map{2, 3};
    const Position blocked{0, 0};
    map.set_blocked(blocked, true);
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {2, 0}, {0, 3}};

    for (const Position invalid : invalid_positions) {
        EXPECT_THROW(static_cast<void>(DStarLite(map, invalid, blocked)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(DStarLite(map, blocked, invalid)),
                     std::out_of_range);
    }
}

TEST(DStarLiteTest, RejectsBlockedEndpointsIncludingStartEqualToGoal) {
    GridMap map{1, 2};
    const Position blocked{0, 0};
    const Position free{0, 1};
    map.set_blocked(blocked, true);

    EXPECT_THROW(static_cast<void>(DStarLite(map, blocked, free)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DStarLite(map, free, blocked)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DStarLite(map, blocked, blocked)),
                 std::invalid_argument);
}

TEST(DStarLiteTest, AcceptsIdenticalUnblockedEndpoints) {
    GridMap map{1, 1};
    const Position position{0, 0};

    const DStarLite planner{map, position, position};

    EXPECT_EQ(planner.start(), position);
    EXPECT_EQ(planner.goal(), position);
}

TEST(DStarLiteTest, AcceptsDisconnectedEndpointsWithoutSearching) {
    GridMap map{1, 3};
    map.set_blocked(Position{0, 1}, true);
    const Position start{0, 0};
    const Position goal{0, 2};

    const DStarLite planner{map, start, goal};

    EXPECT_EQ(planner.start(), start);
    EXPECT_EQ(planner.goal(), goal);
}

TEST(DStarLiteTest, PlanAtGoalReturnsOnePositionWithNoSearchWork) {
    GridMap map{1, 1};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    DStarLite planner{map, Position{0, 0}, Position{0, 0}};

    for (int call = 0; call < 2; ++call) {
        const SearchResult result = planner.plan();
        EXPECT_TRUE(result.found);
        EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}}));
        EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
        EXPECT_EQ(result.stats.expanded_nodes, 0U);
        EXPECT_EQ(result.stats.frontier_pushes, 0U);
    }
}

TEST(DStarLiteTest, BackwardPropagationPreservesForwardDestinationCosts) {
    GridMap map{1, 4};
    const double weights[] = {7.0, 2.0, 0.25, 0.5};
    for (int column = 0; column < 4; ++column) {
        map.set_terrain_weight(Position{0, column}, weights[column]);
    }
    DStarLite forward{map, Position{0, 0}, Position{0, 3}};
    DStarLite reverse{map, Position{0, 3}, Position{0, 0}};

    const SearchResult result = forward.plan();
    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}, {0, 1}, {0, 2}, {0, 3}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 2.75);
    const SearchResult reversed = reverse.plan();
    ASSERT_TRUE(reversed.found);
    EXPECT_EQ(reversed.path, (std::vector<Position>{{0, 3}, {0, 2}, {0, 1}, {0, 0}}));
    EXPECT_DOUBLE_EQ(reversed.total_cost, 9.25);
}

TEST(DStarLiteTest, FindsCheaperLongerRouteWithWeightsBelowOne) {
    GridMap map{2, 3};
    map.set_terrain_weight(Position{0, 1}, 10.0);
    for (int column = 0; column < 3; ++column) {
        map.set_terrain_weight(Position{1, column}, 0.25);
    }
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};

    const SearchResult result = planner.plan();

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.path,
              (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 1.75);
    EXPECT_DOUBLE_EQ(result.total_cost,
                     dijkstra(map, planner.start(), planner.goal()).total_cost);
}

TEST(DStarLiteTest, ReconstructsFourConnectedObstacleDetour) {
    GridMap map{2, 3};
    map.set_blocked(Position{0, 1}, true);
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};

    const SearchResult result = planner.plan();

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.path,
              (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}}));
    EXPECT_DOUBLE_EQ(result.total_cost, 4.0);
}

TEST(DStarLiteTest, UnreachableGoalReturnsFailureAndRetainsCompletedSearch) {
    GridMap map{1, 3};
    map.set_blocked(Position{0, 1}, true);
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};

    const SearchResult first = planner.plan();
    EXPECT_FALSE(first.found);
    EXPECT_TRUE(first.path.empty());
    EXPECT_EQ(first.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_GT(first.stats.expanded_nodes, 0U);
    EXPECT_GT(first.stats.frontier_pushes, 0U);

    const SearchResult second = planner.plan();
    EXPECT_FALSE(second.found);
    EXPECT_TRUE(second.path.empty());
    EXPECT_EQ(second.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(second.stats.expanded_nodes, 0U);
    EXPECT_EQ(second.stats.frontier_pushes, 0U);
}

TEST(DStarLiteTest, RepeatedPlanReturnsSamePathWithoutFurtherSearchWork) {
    GridMap map{4, 5};
    DStarLite planner{map, Position{0, 0}, Position{3, 4}};
    const SearchResult first = planner.plan();
    ASSERT_TRUE(first.found);
    EXPECT_GT(first.stats.expanded_nodes, 0U);
    EXPECT_GT(first.stats.frontier_pushes, 0U);

    const SearchResult second = planner.plan();

    EXPECT_TRUE(second.found);
    EXPECT_EQ(second.path, first.path);
    EXPECT_DOUBLE_EQ(second.total_cost, first.total_cost);
    EXPECT_EQ(second.stats.expanded_nodes, 0U);
    EXPECT_EQ(second.stats.frontier_pushes, 0U);
}

TEST(DStarLiteTest, MatchesDijkstraAndValidPathsOnSeededWeightedMaps) {
    std::mt19937 random{20261023U};
    const double weights[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
    for (int scenario = 0; scenario < 128; ++scenario) {
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
        const SearchResult expected = dijkstra(map, start, goal);
        DStarLite planner{map, start, goal};
        const SearchResult result = planner.plan();
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
        ASSERT_LE(result.path.size(), 35U);
        double path_cost = 0.0;
        for (std::size_t step = 1; step < result.path.size(); ++step) {
            // Also checks adjacency and that neither endpoint is blocked.
            path_cost += movement_cost(map, result.path[step - 1], result.path[step]);
        }
        EXPECT_DOUBLE_EQ(path_cost, result.total_cost);
    }
}

TEST(DStarLiteTest, RepairsDetourAfterBlockingAndUnblockingPathCell) {
    GridMap map{2, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    ASSERT_DOUBLE_EQ(planner.plan().total_cost, 2.0);

    planner.set_blocked(Position{0, 1}, true);
    EXPECT_TRUE(map.is_blocked(Position{0, 1}));
    const SearchResult detour = planner.plan();
    ASSERT_TRUE(detour.found);
    EXPECT_DOUBLE_EQ(detour.total_cost, 4.0);
    EXPECT_EQ(detour.path,
              (std::vector<Position>{{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}}));
    EXPECT_GT(detour.stats.expanded_nodes, 0U);

    planner.set_blocked(Position{0, 1}, false);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
    expect_matches_dijkstra(map, planner);
    const SearchResult repeated = planner.plan();
    EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
    EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
}

TEST(DStarLiteTest, RepairsReachableToUnreachableAndBack) {
    GridMap map{1, 5};
    DStarLite planner{map, Position{0, 0}, Position{0, 4}};
    ASSERT_TRUE(planner.plan().found);
    planner.set_blocked(Position{0, 2}, true);
    const SearchResult failed = planner.plan();
    EXPECT_FALSE(failed.found);
    EXPECT_TRUE(failed.path.empty());
    EXPECT_EQ(failed.total_cost, std::numeric_limits<double>::infinity());
    planner.set_blocked(Position{0, 2}, false);
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 4.0);
}

TEST(DStarLiteTest, UnblocksCheapTerrainWithoutInvalidatingHeuristic) {
    GridMap map{2, 4};
    for (int column = 0; column < 4; ++column) {
        map.set_terrain_weight(Position{1, column}, 0.1);
        map.set_blocked(Position{1, column}, true);
    }
    DStarLite planner{map, Position{0, 0}, Position{0, 3}};
    ASSERT_DOUBLE_EQ(planner.plan().total_cost, 3.0);
    for (int column = 0; column < 4; ++column) {
        planner.set_blocked(Position{1, column}, false);
        expect_matches_dijkstra(map, planner);
    }
    EXPECT_NEAR(planner.plan().total_cost, 1.4, 1e-12);
}

TEST(DStarLiteTest, UpdatesBeforeFirstPlanUseCurrentMap) {
    GridMap map{1, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    planner.set_blocked(Position{0, 1}, true);
    EXPECT_FALSE(planner.plan().found);
    planner.set_blocked(Position{0, 1}, false);
    expect_matches_dijkstra(map, planner);
    EXPECT_TRUE(planner.plan().found);
}

TEST(DStarLiteTest, RejectedAndUnchangedUpdatesPreserveMapAndSearchState) {
    GridMap map{2, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    const SearchResult original = planner.plan();
    for (const Position invalid : {Position{-1, 0}, Position{0, -1},
                                   Position{2, 0}, Position{0, 3}}) {
        EXPECT_THROW(planner.set_blocked(invalid, true), std::out_of_range);
        EXPECT_THROW(planner.set_blocked(invalid, false), std::out_of_range);
    }
    EXPECT_THROW(planner.set_blocked(planner.start(), true), std::invalid_argument);
    EXPECT_THROW(planner.set_blocked(planner.goal(), true), std::invalid_argument);
    planner.set_blocked(planner.start(), false);
    planner.set_blocked(Position{1, 1}, false);
    const SearchResult repeated = planner.plan();
    EXPECT_EQ(repeated.path, original.path);
    EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
    EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
    for (int row = 0; row < 2; ++row) {
        for (int column = 0; column < 3; ++column) {
            EXPECT_FALSE(map.is_blocked(Position{row, column}));
        }
    }
}

TEST(DStarLiteTest, BatchedChangesAndRepeatedTogglesUseFinalMap) {
    GridMap map{3, 5};
    DStarLite planner{map, Position{0, 0}, Position{0, 4}};
    ASSERT_TRUE(planner.plan().found);
    planner.set_blocked(Position{0, 1}, true);
    planner.set_blocked(Position{0, 2}, true);
    planner.set_blocked(Position{0, 1}, false);
    planner.set_blocked(Position{0, 1}, true);
    expect_matches_dijkstra(map, planner);
    planner.set_blocked(Position{0, 1}, false);
    planner.set_blocked(Position{0, 2}, false);
    expect_matches_dijkstra(map, planner);
}

TEST(DStarLiteTest, OffPathChangeCanReuseSearchWithFewerExpansions) {
    GridMap map{3, 20};
    DStarLite planner{map, Position{0, 0}, Position{0, 19}};
    ASSERT_TRUE(planner.plan().found);
    planner.set_blocked(Position{2, 0}, true);
    const SearchResult repaired = planner.plan();
    const SearchResult fresh = dijkstra(map, planner.start(), planner.goal());
    EXPECT_TRUE(repaired.found);
    EXPECT_DOUBLE_EQ(repaired.total_cost, fresh.total_cost);
    EXPECT_LT(repaired.stats.expanded_nodes, fresh.stats.expanded_nodes);
}

TEST(DStarLiteTest, ObstacleChangesPreserveStartEqualToGoalBehavior) {
    GridMap map{1, 2};
    DStarLite planner{map, Position{0, 0}, Position{0, 0}};
    planner.set_blocked(Position{0, 1}, true);
    const SearchResult result = planner.plan();
    EXPECT_TRUE(result.found);
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
    EXPECT_EQ(result.path, (std::vector<Position>{{0, 0}}));
    EXPECT_EQ(result.stats.expanded_nodes, 0U);
    EXPECT_EQ(result.stats.frontier_pushes, 0U);
    EXPECT_THROW(planner.set_blocked(Position{0, 0}, true), std::invalid_argument);
}

TEST(DStarLiteTest, RepeatedWeightedObstacleRepairsMatchFreshDijkstra) {
    std::mt19937 random{20261024U};
    const double weights[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
    for (int scenario = 0; scenario < 32; ++scenario) {
        SCOPED_TRACE(scenario);
        GridMap map{5, 7};
        const Position start{0, 0};
        const Position goal{4, 6};
        for (int row = 0; row < 5; ++row) {
            for (int column = 0; column < 7; ++column) {
                const Position cell{row, column};
                map.set_terrain_weight(cell, weights[random() % 7U]);
                const bool blocked = random() % 4U == 0U;
                map.set_blocked(cell, blocked && cell != start && cell != goal);
            }
        }
        DStarLite planner{map, start, goal};
        expect_matches_dijkstra(map, planner);
        for (int update = 0; update < 48; ++update) {
            SCOPED_TRACE(update);
            const Position cell{static_cast<int>(random() % 5U),
                                static_cast<int>(random() % 7U)};
            if (cell != start && cell != goal) {
                planner.set_blocked(cell, !map.is_blocked(cell));
            }
            expect_matches_dijkstra(map, planner);
        }
    }
}

TEST(DStarLiteTest, MovesStartBeforeFirstPlanWithoutChangingGoal) {
    GridMap map{2, 4};
    DStarLite planner{map, Position{0, 0}, Position{0, 3}};
    planner.move_start(Position{1, 1});
    EXPECT_EQ(planner.start(), (Position{1, 1}));
    EXPECT_EQ(planner.goal(), (Position{0, 3}));
    expect_matches_dijkstra(map, planner);
}

TEST(DStarLiteTest, MovingAlongKnownPathReusesGoalDistances) {
    GridMap map{1, 6};
    DStarLite planner{map, Position{0, 0}, Position{0, 5}};
    ASSERT_TRUE(planner.plan().found);
    for (int column = 1; column < 6; ++column) {
        planner.move_start(Position{0, column});
        const SearchResult result = planner.plan();
        ASSERT_TRUE(result.found);
        EXPECT_EQ(result.path.front(), (Position{0, column}));
        EXPECT_EQ(result.path.back(), planner.goal());
        EXPECT_DOUBLE_EQ(result.total_cost, 5.0 - column);
        EXPECT_EQ(result.stats.expanded_nodes, 0U);
        EXPECT_EQ(result.stats.frontier_pushes, 0U);
    }
}

TEST(DStarLiteTest, MovingOffKnownPathRetainsAndRekeysFrontier) {
    GridMap map{4, 6};
    DStarLite planner{map, Position{0, 0}, Position{0, 5}};
    ASSERT_TRUE(planner.plan().found);
    planner.move_start(Position{3, 0});
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 8.0);
    planner.move_start(Position{0, 1});
    expect_matches_dijkstra(map, planner);
    planner.move_start(Position{3, 2});
    expect_matches_dijkstra(map, planner);
}

TEST(DStarLiteTest, InvalidAndUnchangedStartMovesPreserveSearchState) {
    GridMap map{2, 4};
    map.set_blocked(Position{1, 1}, true);
    DStarLite planner{map, Position{0, 0}, Position{0, 3}};
    const SearchResult original = planner.plan();
    for (const Position invalid : {Position{-1, 0}, Position{0, -1},
                                   Position{2, 0}, Position{0, 4}}) {
        EXPECT_THROW(planner.move_start(invalid), std::out_of_range);
    }
    EXPECT_THROW(planner.move_start(Position{1, 1}), std::invalid_argument);
    planner.move_start(planner.start());
    EXPECT_EQ(planner.start(), (Position{0, 0}));
    const SearchResult repeated = planner.plan();
    EXPECT_EQ(repeated.path, original.path);
    EXPECT_DOUBLE_EQ(repeated.total_cost, original.total_cost);
    EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
    EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
}

TEST(DStarLiteTest, MovingAcrossDisconnectedRegionsChangesReachability) {
    GridMap map{1, 5};
    map.set_blocked(Position{0, 2}, true);
    DStarLite planner{map, Position{0, 0}, Position{0, 4}};
    EXPECT_FALSE(planner.plan().found);
    planner.move_start(Position{0, 3});
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 1.0);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner);
    EXPECT_FALSE(planner.plan().found);
}

TEST(DStarLiteTest, VisitingGoalPreservesPendingObstacleRepairs) {
    GridMap map{2, 4};
    DStarLite planner{map, Position{0, 0}, Position{0, 3}};
    ASSERT_TRUE(planner.plan().found);
    planner.move_start(planner.goal());
    planner.set_blocked(Position{0, 1}, true);
    const SearchResult at_goal = planner.plan();
    EXPECT_TRUE(at_goal.found);
    EXPECT_EQ(at_goal.path, (std::vector<Position>{{0, 3}}));
    EXPECT_DOUBLE_EQ(at_goal.total_cost, 0.0);
    EXPECT_EQ(at_goal.stats.expanded_nodes, 0U);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 5.0);
}

TEST(DStarLiteTest, InitiallyAtGoalCanMoveAwayAndStartSearching) {
    GridMap map{1, 4};
    DStarLite planner{map, Position{0, 3}, Position{0, 3}};
    EXPECT_TRUE(planner.plan().found);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 3.0);
}

TEST(DStarLiteTest, ProtectsCurrentStartAndAllowsBlockingFormerStart) {
    GridMap map{2, 4};
    DStarLite planner{map, Position{0, 0}, Position{0, 3}};
    ASSERT_TRUE(planner.plan().found);
    planner.move_start(Position{0, 1});
    planner.set_blocked(Position{0, 0}, true);
    EXPECT_TRUE(map.is_blocked(Position{0, 0}));
    EXPECT_THROW(planner.set_blocked(Position{0, 1}, true), std::invalid_argument);
    EXPECT_THROW(planner.set_blocked(planner.goal(), true), std::invalid_argument);
    expect_matches_dijkstra(map, planner);
}

TEST(DStarLiteTest, MixedMovesAndObstacleChangesMatchDijkstraOnWeightedMaps) {
    std::mt19937 random{20261025U};
    const double weights[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
    for (int scenario = 0; scenario < 24; ++scenario) {
        SCOPED_TRACE(scenario);
        GridMap map{5, 7};
        const Position goal{4, 6};
        for (int row = 0; row < 5; ++row) {
            for (int column = 0; column < 7; ++column) {
                const Position cell{row, column};
                map.set_terrain_weight(cell, weights[random() % 7U]);
                const bool blocked = random() % 4U == 0U;
                map.set_blocked(cell, blocked && cell != Position{0, 0} && cell != goal);
            }
        }
        DStarLite planner{map, Position{0, 0}, goal};
        expect_matches_dijkstra(map, planner);
        for (int round = 0; round < 40; ++round) {
            SCOPED_TRACE(round);
            // Mix several changes before planning, including repeated moves.
            for (int operation = 0; operation < 3; ++operation) {
                const Position cell{static_cast<int>(random() % 5U),
                                    static_cast<int>(random() % 7U)};
                if (random() % 2U == 0U) {
                    if (!map.is_blocked(cell)) {
                        planner.move_start(cell);
                    }
                } else if (cell != planner.start() && cell != goal) {
                    planner.set_blocked(cell, !map.is_blocked(cell));
                }
            }
            expect_matches_dijkstra(map, planner);
            const SearchResult repeated = planner.plan();
            EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
            EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
            EXPECT_EQ(planner.goal(), goal);
        }
    }
}

TEST(DStarLiteTest, DefaultConnectivityMatchesExplicitFour) {
    GridMap map{3, 4};
    DStarLite default_planner{map, Position{0, 0}, Position{2, 3}};
    DStarLite four_planner{map, Position{0, 0}, Position{2, 3}, Connectivity::Four};
    const SearchResult result = default_planner.plan();
    const SearchResult explicit_four = four_planner.plan();
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, explicit_four.path);
    EXPECT_DOUBLE_EQ(result.total_cost, 5.0);
    EXPECT_EQ(result.stats.expanded_nodes, explicit_four.stats.expanded_nodes);
    EXPECT_EQ(result.stats.frontier_pushes, explicit_four.stats.frontier_pushes);
}

TEST(DStarLiteTest, EightConnectivityCombinesDiagonalsAndStraightMoves) {
    GridMap map{3, 4};
    DStarLite planner{map, Position{0, 0}, Position{2, 3}, Connectivity::Eight};
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    const SearchResult repeated = planner.plan();
    EXPECT_NEAR(repeated.total_cost, 2.0 * std::sqrt(2.0) + 1.0, 1e-12);
    EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
    EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
}

TEST(DStarLiteTest, EightConnectivityPreservesWeightedForwardDiagonalCosts) {
    GridMap map{2, 2};
    map.set_terrain_weight(Position{0, 0}, 7.0);
    map.set_terrain_weight(Position{1, 1}, 0.25);
    map.set_terrain_weight(Position{0, 1}, 10.0);
    map.set_terrain_weight(Position{1, 0}, 10.0);
    DStarLite forward{map, Position{0, 0}, Position{1, 1}, Connectivity::Eight};
    DStarLite reverse{map, Position{1, 1}, Position{0, 0}, Connectivity::Eight};
    expect_matches_dijkstra(map, forward, Connectivity::Eight);
    expect_matches_dijkstra(map, reverse, Connectivity::Eight);
    EXPECT_NEAR(forward.plan().total_cost, 0.25 * std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(reverse.plan().total_cost, 7.0 * std::sqrt(2.0), 1e-12);
}

TEST(DStarLiteTest, EightConnectivityRepairsDiagonalWhenOffPathSideCellChanges) {
    GridMap map{2, 2};
    DStarLite planner{map, Position{0, 0}, Position{1, 1}, Connectivity::Eight};
    const SearchResult initial = planner.plan();
    ASSERT_TRUE(initial.found);
    ASSERT_EQ(initial.path, (std::vector<Position>{{0, 0}, {1, 1}}));

    // Neither diagonal endpoint changes, but a side cell blocks that edge.
    planner.set_blocked(Position{0, 1}, true);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
    planner.set_blocked(Position{1, 0}, true);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    EXPECT_FALSE(planner.plan().found);
    planner.set_blocked(Position{0, 1}, false);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
    planner.set_blocked(Position{1, 0}, false);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    EXPECT_NEAR(planner.plan().total_cost, std::sqrt(2.0), 1e-12);
}

TEST(DStarLiteTest, EightConnectivityRepairsCornerChangeAfterMovingStart) {
    GridMap map{4, 5};
    DStarLite planner{map, Position{0, 0}, Position{3, 4}, Connectivity::Eight};
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    planner.move_start(Position{1, 1});
    planner.set_blocked(Position{1, 2}, true);
    planner.set_blocked(Position{2, 1}, true);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    planner.move_start(Position{0, 4});
    planner.set_blocked(Position{1, 2}, false);
    planner.set_blocked(Position{2, 1}, false);
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
    planner.move_start(planner.goal());
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 0.0);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
}

TEST(DStarLiteTest, EightConnectivityMixedWeightedUpdatesMatchDijkstra) {
    std::mt19937 random{20261026U};
    const double weights[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
    for (int scenario = 0; scenario < 24; ++scenario) {
        SCOPED_TRACE(scenario);
        GridMap map{5, 7};
        const Position goal{4, 6};
        for (int row = 0; row < 5; ++row) {
            for (int column = 0; column < 7; ++column) {
                const Position cell{row, column};
                map.set_terrain_weight(cell, weights[random() % 7U]);
                const bool blocked = random() % 4U == 0U;
                map.set_blocked(cell, blocked && cell != Position{0, 0} && cell != goal);
            }
        }
        DStarLite planner{map, Position{0, 0}, goal, Connectivity::Eight};
        expect_matches_dijkstra(map, planner, Connectivity::Eight);
        for (int round = 0; round < 40; ++round) {
            SCOPED_TRACE(round);
            for (int operation = 0; operation < 3; ++operation) {
                const Position cell{static_cast<int>(random() % 5U),
                                    static_cast<int>(random() % 7U)};
                if (random() % 2U == 0U) {
                    if (!map.is_blocked(cell)) {
                        planner.move_start(cell);
                    }
                } else if (cell != planner.start() && cell != goal) {
                    planner.set_blocked(cell, !map.is_blocked(cell));
                }
            }
            expect_matches_dijkstra(map, planner, Connectivity::Eight);
            const SearchResult repeated = planner.plan();
            EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
            EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
        }
    }
}

TEST(DStarLiteTest, RepairsRoutesAfterTerrainBecomesExpensiveAndCheap) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        GridMap map{2, 3};
        DStarLite planner{map, Position{0, 0}, Position{0, 2}, connectivity};
        ASSERT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
        planner.set_terrain_weight(Position{0, 1}, 10.0);
        EXPECT_DOUBLE_EQ(map.terrain_weight(Position{0, 1}), 10.0);
        expect_matches_dijkstra(map, planner, connectivity);
        EXPECT_GT(planner.plan().total_cost, 2.0);
        planner.set_terrain_weight(Position{0, 1}, 1.0);
        EXPECT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
        for (int column = 0; column < 3; ++column) {
            planner.set_terrain_weight(Position{1, column}, 0.1);
        }
        expect_matches_dijkstra(map, planner, connectivity);
        EXPECT_LT(planner.plan().total_cost, 2.0);
    }
}

TEST(DStarLiteTest, WeightUpdatesPreserveDestinationCostConvention) {
    GridMap map{1, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    ASSERT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
    planner.set_terrain_weight(planner.start(), 7.0);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 2.0);
    planner.set_terrain_weight(planner.goal(), 3.0);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 4.0);
    planner.move_start(Position{0, 1});
    planner.set_terrain_weight(planner.start(), 0.05);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 3.0);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner);
    EXPECT_NEAR(planner.plan().total_cost, 3.05, 1e-12);
}

TEST(DStarLiteTest, LowerBoundChangesAfterStartMovesRetainValidSearch) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        GridMap map{3, 6};
        DStarLite planner{map, Position{0, 0}, Position{0, 5}, connectivity};
        ASSERT_TRUE(planner.plan().found);
        planner.move_start(Position{2, 0});
        expect_matches_dijkstra(map, planner, connectivity);
        for (int column = 0; column < 6; ++column) {
            planner.set_terrain_weight(Position{1, column}, 0.05);
        }
        // The new bound also changes how the accumulated km must be handled.
        planner.move_start(Position{0, 1});
        expect_matches_dijkstra(map, planner, connectivity);
        planner.move_start(Position{2, 2});
        expect_matches_dijkstra(map, planner, connectivity);
        planner.set_terrain_weight(Position{1, 3}, 20.0);
        expect_matches_dijkstra(map, planner, connectivity);
        const SearchResult repeated = planner.plan();
        EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
        EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
    }
}

TEST(DStarLiteTest, BlockedTerrainWeightCanChangeBeforeUnblocking) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        GridMap map{2, 3};
        map.set_blocked(Position{1, 1}, true);
        DStarLite planner{map, Position{0, 0}, Position{0, 2}, connectivity};
        ASSERT_TRUE(planner.plan().found);
        planner.set_terrain_weight(Position{1, 1}, 0.01);
        EXPECT_TRUE(map.is_blocked(Position{1, 1}));
        expect_matches_dijkstra(map, planner, connectivity);
        planner.set_blocked(Position{1, 1}, false);
        planner.set_terrain_weight(Position{0, 1}, 10.0);
        expect_matches_dijkstra(map, planner, connectivity);
    }
}

TEST(DStarLiteTest, WeightUpdatesBeforeInitialPlanUseFinalMap) {
    GridMap map{1, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    planner.set_terrain_weight(Position{0, 1}, 0.25);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 1.25);
    // A temporary low weight is restored before planning.
    planner.set_terrain_weight(Position{0, 1}, 0.01);
    planner.set_terrain_weight(Position{0, 1}, 0.25);
    expect_matches_dijkstra(map, planner);
    EXPECT_DOUBLE_EQ(planner.plan().total_cost, 1.25);
}

TEST(DStarLiteTest, RejectedAndUnchangedWeightsPreserveMapAndSearch) {
    GridMap map{2, 3};
    DStarLite planner{map, Position{0, 0}, Position{0, 2}};
    const SearchResult original = planner.plan();
    const double invalid_weights[] = {
        0.0, -1.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    for (const double invalid : invalid_weights) {
        EXPECT_THROW(planner.set_terrain_weight(Position{0, 1}, invalid),
                     std::invalid_argument);
        EXPECT_THROW(planner.set_terrain_weight(Position{-1, 0}, invalid),
                     std::out_of_range);
    }
    for (const Position invalid : {Position{-1, 0}, Position{0, -1},
                                   Position{2, 0}, Position{0, 3}}) {
        EXPECT_THROW(planner.set_terrain_weight(invalid, 1.0), std::out_of_range);
    }
    planner.set_terrain_weight(Position{0, 1}, 1.0);
    const SearchResult repeated = planner.plan();
    EXPECT_EQ(repeated.path, original.path);
    EXPECT_DOUBLE_EQ(repeated.total_cost, original.total_cost);
    EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
    EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
    for (int row = 0; row < 2; ++row) {
        for (int column = 0; column < 3; ++column) {
            EXPECT_DOUBLE_EQ(map.terrain_weight(Position{row, column}), 1.0);
        }
    }
}

TEST(DStarLiteTest, AtGoalRetainsPendingWeightAndLowerBoundChanges) {
    GridMap map{2, 4};
    DStarLite planner{map, Position{0, 0}, Position{0, 3}, Connectivity::Eight};
    ASSERT_TRUE(planner.plan().found);
    planner.move_start(planner.goal());
    planner.set_terrain_weight(Position{0, 1}, 0.01);
    planner.set_terrain_weight(planner.goal(), 5.0);
    const SearchResult at_goal = planner.plan();
    EXPECT_TRUE(at_goal.found);
    EXPECT_DOUBLE_EQ(at_goal.total_cost, 0.0);
    EXPECT_EQ(at_goal.stats.expanded_nodes, 0U);
    EXPECT_EQ(at_goal.stats.frontier_pushes, 0U);
    planner.move_start(Position{0, 0});
    expect_matches_dijkstra(map, planner, Connectivity::Eight);
}

TEST(DStarLiteTest, MixedTerrainObstacleAndStartUpdatesMatchDijkstra) {
    std::mt19937 random{20261027U};
    const double weights[] = {0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 20.0};
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        SCOPED_TRACE(static_cast<int>(connectivity));
        for (int scenario = 0; scenario < 16; ++scenario) {
            SCOPED_TRACE(scenario);
            GridMap map{5, 7};
            const Position goal{4, 6};
            DStarLite planner{map, Position{0, 0}, goal, connectivity};
            expect_matches_dijkstra(map, planner, connectivity);
            for (int round = 0; round < 40; ++round) {
                SCOPED_TRACE(round);
                for (int operation = 0; operation < 3; ++operation) {
                    const Position cell{static_cast<int>(random() % 5U),
                                        static_cast<int>(random() % 7U)};
                    switch (random() % 3U) {
                    case 0:
                        if (!map.is_blocked(cell)) {
                            planner.move_start(cell);
                        }
                        break;
                    case 1:
                        if (cell != planner.start() && cell != goal) {
                            planner.set_blocked(cell, !map.is_blocked(cell));
                        }
                        break;
                    default:
                        planner.set_terrain_weight(cell, weights[random() % 9U]);
                        break;
                    }
                }
                expect_matches_dijkstra(map, planner, connectivity);
                const SearchResult repeated = planner.plan();
                EXPECT_EQ(repeated.stats.expanded_nodes, 0U);
                EXPECT_EQ(repeated.stats.frontier_pushes, 0U);
            }
        }
    }
}

}  // namespace
}  // namespace dynamic_route
