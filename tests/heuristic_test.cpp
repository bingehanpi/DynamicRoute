#include "dynamic_route/heuristic.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(HeuristicTest, AutomaticFourUsesManhattanDistance) {
    EXPECT_DOUBLE_EQ(heuristic_distance(Position{1, 2}, Position{4, 6},
                                       Connectivity::Four, Heuristic::Automatic), 7.0);
}

TEST(HeuristicTest, AutomaticEightCombinesThreeDiagonalsAndOneStraightStep) {
    EXPECT_NEAR(heuristic_distance(Position{1, 2}, Position{4, 6},
                                  Connectivity::Eight, Heuristic::Automatic),
                3.0 * std::sqrt(2.0) + 1.0, 1e-12);
}

TEST(HeuristicTest, EuclideanUsesCoordinateDifferencesInBothConnectivityModes) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        EXPECT_DOUBLE_EQ(heuristic_distance(Position{1, 2}, Position{4, 6},
                                           connectivity, Heuristic::Euclidean), 5.0);
        EXPECT_DOUBLE_EQ(heuristic_distance(Position{4, 6}, Position{1, 2},
                                           connectivity, Heuristic::Euclidean), 5.0);
    }
}

TEST(HeuristicTest, ZeroIgnoresDistanceAndConnectivity) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        EXPECT_DOUBLE_EQ(heuristic_distance(Position{-3, -4}, Position{8, 9},
                                           connectivity, Heuristic::Zero), 0.0);
    }
}

TEST(HeuristicTest, IdenticalPositionsHaveZeroDistanceForEveryOption) {
    for (const Connectivity connectivity : {Connectivity::Four, Connectivity::Eight}) {
        for (const Heuristic heuristic_type :
             {Heuristic::Automatic, Heuristic::Euclidean, Heuristic::Zero}) {
            EXPECT_DOUBLE_EQ(heuristic_distance(Position{2, 3}, Position{2, 3},
                                               connectivity, heuristic_type), 0.0);
        }
    }
}

TEST(HeuristicTest, SupportsExtremeIntegerCoordinatesWithoutOverflow) {
    const int low = std::numeric_limits<int>::min();
    const int high = std::numeric_limits<int>::max();
    const double span = static_cast<double>(high) - low;
    EXPECT_DOUBLE_EQ(heuristic_distance(Position{low, low}, Position{high, high},
                                       Connectivity::Four, Heuristic::Automatic),
                     2.0 * span);
}

TEST(HeuristicTest, MinimumWeightIgnoresObstaclesAndReadsCurrentMap) {
    GridMap map{2, 3};
    EXPECT_DOUBLE_EQ(minimum_unblocked_terrain_weight(map), 1.0);
    map.set_terrain_weight(Position{0, 0}, 0.5);
    map.set_terrain_weight(Position{1, 2}, 0.1);
    map.set_blocked(Position{1, 2}, true);
    EXPECT_DOUBLE_EQ(minimum_unblocked_terrain_weight(map), 0.5);

    map.set_blocked(Position{1, 2}, false);
    EXPECT_DOUBLE_EQ(minimum_unblocked_terrain_weight(map), 0.1);
    map.set_terrain_weight(Position{1, 2}, 2.0);
    EXPECT_DOUBLE_EQ(minimum_unblocked_terrain_weight(map), 0.5);
}

TEST(HeuristicTest, FullyBlockedMapHasInfiniteMinimumWeight) {
    GridMap map{1, 1};
    map.set_blocked(Position{0, 0}, true);
    EXPECT_EQ(minimum_unblocked_terrain_weight(map),
              std::numeric_limits<double>::infinity());
}

}  // namespace
}  // namespace dynamic_route
