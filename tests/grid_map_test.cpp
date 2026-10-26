#include "dynamic_route/grid_map.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(GridMapTest, StoresItsDimensions) {
    const GridMap map{3, 5};

    EXPECT_EQ(map.row_count(), 3);
    EXPECT_EQ(map.column_count(), 5);
}

TEST(GridMapTest, RejectsNonPositiveDimensions) {
    EXPECT_THROW((GridMap{0, 5}), std::invalid_argument);
    EXPECT_THROW((GridMap{3, 0}), std::invalid_argument);
    EXPECT_THROW((GridMap{-1, 5}), std::invalid_argument);
    EXPECT_THROW((GridMap{3, -1}), std::invalid_argument);
}

TEST(GridMapTest, ContainsPositionsInsideItsBounds) {
    const GridMap map{3, 5};

    EXPECT_TRUE(map.contains(Position{0, 0}));
    EXPECT_TRUE(map.contains(Position{1, 2}));
    EXPECT_TRUE(map.contains(Position{2, 4}));
}

TEST(GridMapTest, RejectsPositionsOutsideItsBounds) {
    const GridMap map{3, 5};

    EXPECT_FALSE(map.contains(Position{-1, 0}));
    EXPECT_FALSE(map.contains(Position{0, -1}));
    EXPECT_FALSE(map.contains(Position{3, 0}));
    EXPECT_FALSE(map.contains(Position{0, 5}));
}

TEST(GridMapTest, AllCellsAreInitiallyUnblocked) {
    const GridMap map{3, 5};

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            EXPECT_FALSE(map.is_blocked(Position{row, column}));
        }
    }
}

TEST(GridMapTest, CanAddAndRemoveAnObstacle) {
    GridMap map{3, 5};
    const Position obstacle{1, 2};

    map.set_blocked(obstacle, true);
    EXPECT_TRUE(map.is_blocked(obstacle));
    EXPECT_TRUE(map.contains(obstacle));

    map.set_blocked(obstacle, false);
    EXPECT_FALSE(map.is_blocked(obstacle));
}

TEST(GridMapTest, UpdatingOneCellLeavesEveryOtherCellUnchanged) {
    GridMap map{3, 5};
    const Position obstacle{1, 2};
    map.set_blocked(obstacle, true);

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            const Position position{row, column};
            EXPECT_EQ(map.is_blocked(position), position == obstacle);
        }
    }
}

TEST(GridMapTest, DistinguishesCellsOnOppositeSidesOfARowBoundary) {
    GridMap map{3, 5};

    map.set_blocked(Position{0, 4}, true);
    map.set_blocked(Position{2, 4}, true);

    EXPECT_TRUE(map.is_blocked(Position{0, 4}));
    EXPECT_FALSE(map.is_blocked(Position{1, 0}));
    EXPECT_TRUE(map.is_blocked(Position{2, 4}));
    EXPECT_FALSE(map.is_blocked(Position{2, 3}));
}

TEST(GridMapTest, RejectsOutOfBoundsObstacleAccessWithoutChangingCells) {
    GridMap map{3, 5};
    map.set_blocked(Position{1, 2}, true);
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {3, 0}, {0, 5}};

    for (const Position position : invalid_positions) {
        EXPECT_THROW(static_cast<void>(map.is_blocked(position)), std::out_of_range);
        EXPECT_THROW(map.set_blocked(position, true), std::out_of_range);
        EXPECT_THROW(map.set_blocked(position, false), std::out_of_range);
    }

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            const Position position{row, column};
            EXPECT_EQ(map.is_blocked(position), (position == Position{1, 2}));
        }
    }
}

TEST(GridMapTest, SupportsASingleCellMapAndRepeatedUpdates) {
    GridMap map{1, 1};
    const Position position{0, 0};

    map.set_blocked(position, true);
    map.set_blocked(position, true);
    EXPECT_TRUE(map.is_blocked(position));

    map.set_blocked(position, false);
    map.set_blocked(position, false);
    EXPECT_FALSE(map.is_blocked(position));
}

TEST(GridMapTest, AllTerrainWeightsInitiallyEqualOne) {
    const GridMap map{3, 5};

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            EXPECT_DOUBLE_EQ(map.terrain_weight(Position{row, column}), 1.0);
        }
    }
}

TEST(GridMapTest, CanUpdateTerrainWeightsIndependently) {
    GridMap map{3, 5};
    map.set_terrain_weight(Position{0, 4}, 2.5);
    map.set_terrain_weight(Position{1, 0}, 0.5);
    map.set_terrain_weight(Position{2, 4}, 3.0);

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            const Position position{row, column};
            double expected = 1.0;
            if (position == Position{0, 4}) {
                expected = 2.5;
            } else if (position == Position{1, 0}) {
                expected = 0.5;
            } else if (position == Position{2, 4}) {
                expected = 3.0;
            }
            EXPECT_DOUBLE_EQ(map.terrain_weight(position), expected);
        }
    }

    map.set_terrain_weight(Position{0, 4}, 1.0);
    EXPECT_DOUBLE_EQ(map.terrain_weight(Position{0, 4}), 1.0);
}

TEST(GridMapTest, RejectsInvalidWeightsWithoutChangingPreviousWeight) {
    GridMap map{1, 1};
    const Position position{0, 0};
    map.set_terrain_weight(position, 2.5);
    const double invalid_weights[] = {
        0.0, -0.0, -1.0,
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };

    for (const double weight : invalid_weights) {
        EXPECT_THROW(map.set_terrain_weight(position, weight), std::invalid_argument);
        EXPECT_DOUBLE_EQ(map.terrain_weight(position), 2.5);
    }
}

TEST(GridMapTest, RejectsOutOfBoundsTerrainAccess) {
    GridMap map{3, 5};
    const Position invalid_positions[] = {{-1, 0}, {0, -1}, {3, 0}, {0, 5}};

    for (const Position position : invalid_positions) {
        EXPECT_THROW(static_cast<void>(map.terrain_weight(position)), std::out_of_range);
        EXPECT_THROW(map.set_terrain_weight(position, 2.5), std::out_of_range);
        // Position validation takes priority when both inputs are invalid.
        EXPECT_THROW(map.set_terrain_weight(position, 0.0), std::out_of_range);
    }

    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            EXPECT_DOUBLE_EQ(map.terrain_weight(Position{row, column}), 1.0);
        }
    }
}

TEST(GridMapTest, TerrainWeightAndObstacleStateAreIndependent) {
    GridMap map{1, 1};
    const Position position{0, 0};

    map.set_terrain_weight(position, 2.5);
    EXPECT_FALSE(map.is_blocked(position));

    map.set_blocked(position, true);
    EXPECT_DOUBLE_EQ(map.terrain_weight(position), 2.5);

    map.set_terrain_weight(position, 0.5);
    EXPECT_TRUE(map.is_blocked(position));

    map.set_blocked(position, false);
    EXPECT_DOUBLE_EQ(map.terrain_weight(position), 0.5);
}

}  // namespace
}  // namespace dynamic_route
