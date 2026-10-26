#include "dynamic_route/movement.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(MovementTest, ReturnsFourNeighborsInFixedOrder) {
    const GridMap map{3, 5};
    const std::vector<Position> expected{{0, 2}, {2, 2}, {1, 1}, {1, 3}};

    EXPECT_EQ(four_connected_neighbors(map, Position{1, 2}), expected);
}

TEST(MovementTest, FiltersNeighborsOutsideTheMap) {
    const GridMap map{3, 5};
    const std::vector<Position> top_left{{1, 0}, {0, 1}};
    const std::vector<Position> bottom_right{{1, 4}, {2, 3}};

    EXPECT_EQ(four_connected_neighbors(map, Position{0, 0}), top_left);
    EXPECT_EQ(four_connected_neighbors(map, Position{2, 4}), bottom_right);
}

TEST(MovementTest, ReadsCurrentObstacleStateOnEachCall) {
    GridMap map{3, 3};
    const Position origin{1, 1};
    map.set_blocked(Position{0, 1}, true);
    map.set_blocked(Position{1, 2}, true);
    const std::vector<Position> expected{{2, 1}, {1, 0}};
    EXPECT_EQ(four_connected_neighbors(map, origin), expected);

    map.set_blocked(Position{0, 1}, false);
    const std::vector<Position> updated{{0, 1}, {2, 1}, {1, 0}};
    EXPECT_EQ(four_connected_neighbors(map, origin), updated);
}

TEST(MovementTest, BlockedOrIsolatedOriginsHaveNoNeighbors) {
    GridMap map{3, 3};
    const Position origin{1, 1};
    map.set_blocked(origin, true);
    EXPECT_TRUE(four_connected_neighbors(map, origin).empty());

    map.set_blocked(origin, false);
    for (const Position neighbor : four_connected_neighbors(map, origin)) {
        map.set_blocked(neighbor, true);
    }
    EXPECT_TRUE(four_connected_neighbors(map, origin).empty());

    EXPECT_TRUE(four_connected_neighbors(GridMap{1, 1}, Position{0, 0}).empty());
}

TEST(MovementTest, SupportsSingleRowAndSingleColumnMaps) {
    const std::vector<Position> horizontal{{0, 0}, {0, 2}};
    const std::vector<Position> vertical{{0, 0}, {2, 0}};

    EXPECT_EQ(four_connected_neighbors(GridMap{1, 3}, Position{0, 1}), horizontal);
    EXPECT_EQ(four_connected_neighbors(GridMap{3, 1}, Position{1, 0}), vertical);
}

TEST(MovementTest, RejectsOutOfBoundsOrigins) {
    const GridMap map{3, 5};
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {3, 0}, {0, 5}};

    for (const Position position : invalid_positions) {
        EXPECT_THROW(static_cast<void>(four_connected_neighbors(map, position)),
                     std::out_of_range);
    }
}

TEST(MovementTest, EveryReturnedNeighborHasTheDestinationWeightAsCost) {
    GridMap map{3, 3};
    const Position origin{1, 1};
    map.set_terrain_weight(origin, 7.0);
    map.set_terrain_weight(Position{0, 1}, 2.5);
    map.set_terrain_weight(Position{2, 1}, 0.5);

    for (const Position neighbor : four_connected_neighbors(map, origin)) {
        EXPECT_DOUBLE_EQ(movement_cost(map, origin, neighbor),
                         map.terrain_weight(neighbor));
        EXPECT_DOUBLE_EQ(movement_cost(map, neighbor, origin), 7.0);
    }
}

TEST(MovementTest, ReadsCurrentTerrainWeightOnEachCall) {
    GridMap map{1, 2};
    EXPECT_DOUBLE_EQ(movement_cost(map, Position{0, 0}, Position{0, 1}), 1.0);

    map.set_terrain_weight(Position{0, 1}, 2.5);
    EXPECT_DOUBLE_EQ(movement_cost(map, Position{0, 0}, Position{0, 1}), 2.5);
}

TEST(MovementTest, RejectsOutOfBoundsMovementEndpoints) {
    const GridMap map{3, 5};
    const Position valid{0, 0};
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {3, 0}, {0, 5}};

    for (const Position position : invalid_positions) {
        EXPECT_THROW(static_cast<void>(movement_cost(map, position, valid)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(movement_cost(map, valid, position)),
                     std::out_of_range);
    }
}

TEST(MovementTest, RejectsStationaryDiagonalAndLongMoves) {
    const GridMap map{3, 5};
    const Position from{0, 0};
    const Position invalid_destinations[] = {{0, 0}, {1, 1}, {0, 2}, {2, 0}};

    for (const Position to : invalid_destinations) {
        EXPECT_THROW(static_cast<void>(movement_cost(map, from, to)),
                     std::invalid_argument);
    }
}

TEST(MovementTest, RejectsMovementIntoOrOutOfObstacles) {
    GridMap map{1, 2};
    const Position from{0, 0};
    const Position to{0, 1};

    map.set_blocked(to, true);
    EXPECT_THROW(static_cast<void>(movement_cost(map, from, to)),
                 std::invalid_argument);

    map.set_blocked(to, false);
    map.set_blocked(from, true);
    EXPECT_THROW(static_cast<void>(movement_cost(map, from, to)),
                 std::invalid_argument);
}

TEST(EightConnectedMovementTest, ReturnsEightNeighborsInFixedOrder) {
    const GridMap map{3, 3};
    const std::vector<Position> expected{
        {0, 1}, {2, 1}, {1, 0}, {1, 2}, {0, 0}, {0, 2}, {2, 0}, {2, 2}};

    EXPECT_EQ(eight_connected_neighbors(map, Position{1, 1}), expected);
}

TEST(EightConnectedMovementTest, FiltersNeighborsAtMapCorners) {
    const GridMap map{3, 5};
    const std::vector<Position> top_left{{1, 0}, {0, 1}, {1, 1}};
    const std::vector<Position> bottom_right{{1, 4}, {2, 3}, {1, 3}};

    EXPECT_EQ(eight_connected_neighbors(map, Position{0, 0}), top_left);
    EXPECT_EQ(eight_connected_neighbors(map, Position{2, 4}), bottom_right);
}

TEST(EightConnectedMovementTest, RejectsCornerCuttingForEveryDiagonalDirection) {
    GridMap map{3, 3};
    const Position origin{1, 1};
    const Position diagonals[] = {{0, 0}, {0, 2}, {2, 0}, {2, 2}};
    for (const Position diagonal : diagonals) {
        const Position sides[] = {{origin.row, diagonal.column},
                                  {diagonal.row, origin.column}};
        // Check each side alone, both sides, and neither side blocked.
        for (int mask = 0; mask < 4; ++mask) {
            map.set_blocked(sides[0], (mask & 1) != 0);
            map.set_blocked(sides[1], (mask & 2) != 0);
            const auto neighbors = eight_connected_neighbors(map, origin);
            const bool included = std::find(neighbors.begin(), neighbors.end(),
                                             diagonal) != neighbors.end();
            EXPECT_EQ(included, mask == 0);
            if (mask == 0) {
                EXPECT_DOUBLE_EQ(movement_cost(map, origin, diagonal,
                                                Connectivity::Eight),
                                 std::sqrt(2.0));
            } else {
                EXPECT_THROW(static_cast<void>(movement_cost(
                                 map, origin, diagonal, Connectivity::Eight)),
                             std::invalid_argument);
            }
        }
        map.set_blocked(sides[0], false);
        map.set_blocked(sides[1], false);
    }
}

TEST(EightConnectedMovementTest, FiltersBlockedDestinationsAndOrigins) {
    GridMap map{2, 2};
    const Position origin{0, 0};
    const Position diagonal{1, 1};
    map.set_blocked(diagonal, true);
    EXPECT_EQ(eight_connected_neighbors(map, origin),
              (std::vector<Position>{{1, 0}, {0, 1}}));
    EXPECT_THROW(static_cast<void>(movement_cost(
                     map, origin, diagonal, Connectivity::Eight)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(movement_cost(
                     map, diagonal, origin, Connectivity::Eight)),
                 std::invalid_argument);

    map.set_blocked(origin, true);
    EXPECT_TRUE(eight_connected_neighbors(map, origin).empty());
}

TEST(EightConnectedMovementTest, SupportsSingleCellRowAndColumnMaps) {
    EXPECT_TRUE(eight_connected_neighbors(GridMap{1, 1}, Position{0, 0}).empty());
    EXPECT_EQ(eight_connected_neighbors(GridMap{1, 3}, Position{0, 1}),
              (std::vector<Position>{{0, 0}, {0, 2}}));
    EXPECT_EQ(eight_connected_neighbors(GridMap{3, 1}, Position{1, 0}),
              (std::vector<Position>{{0, 0}, {2, 0}}));
}

TEST(EightConnectedMovementTest, RejectsOutOfBoundsOriginsAndMovementEndpoints) {
    const GridMap map{3, 5};
    const Position valid{0, 0};
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {3, 0}, {0, 5}};
    for (const Position invalid : invalid_positions) {
        EXPECT_THROW(static_cast<void>(eight_connected_neighbors(map, invalid)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(movement_cost(
                         map, invalid, valid, Connectivity::Eight)),
                     std::out_of_range);
        EXPECT_THROW(static_cast<void>(movement_cost(
                         map, valid, invalid, Connectivity::Eight)),
                     std::out_of_range);
    }
}

TEST(EightConnectedMovementTest, UsesStepLengthTimesCurrentDestinationWeight) {
    GridMap map{3, 3};
    const Position origin{1, 1};
    map.set_terrain_weight(origin, 7.0);
    for (const Position neighbor : eight_connected_neighbors(map, origin)) {
        map.set_terrain_weight(neighbor, 0.5);
        const bool diagonal = neighbor.row != origin.row &&
                              neighbor.column != origin.column;
        const double length = diagonal ? std::sqrt(2.0) : 1.0;
        EXPECT_DOUBLE_EQ(movement_cost(map, origin, neighbor, Connectivity::Eight),
                         length * 0.5);
        EXPECT_DOUBLE_EQ(movement_cost(map, neighbor, origin, Connectivity::Eight),
                         length * 7.0);
        map.set_terrain_weight(neighbor, 2.5);
        EXPECT_DOUBLE_EQ(movement_cost(map, origin, neighbor, Connectivity::Eight),
                         length * 2.5);
    }
}

TEST(EightConnectedMovementTest, RejectsStationaryAndLongMoves) {
    const GridMap map{3, 3};
    const Position origin{0, 0};
    const Position invalid_destinations[] = {{0, 0}, {0, 2}, {2, 0}, {1, 2}, {2, 2}};
    for (const Position destination : invalid_destinations) {
        EXPECT_THROW(static_cast<void>(movement_cost(
                         map, origin, destination, Connectivity::Eight)),
                     std::invalid_argument);
    }
}

TEST(EightConnectedMovementTest, FourConnectivityStillRejectsDiagonals) {
    const GridMap map{2, 2};
    const Position origin{0, 0};
    const Position diagonal{1, 1};
    EXPECT_THROW(static_cast<void>(movement_cost(map, origin, diagonal)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(movement_cost(
                     map, origin, diagonal, Connectivity::Four)),
                 std::invalid_argument);
    EXPECT_EQ(four_connected_neighbors(map, origin),
              (std::vector<Position>{{1, 0}, {0, 1}}));
}

}  // namespace
}  // namespace dynamic_route
