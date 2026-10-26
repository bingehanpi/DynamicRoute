#include "dynamic_route/position.hpp"

#include <unordered_set>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(PositionTest, DefaultsToOrigin) {
    const Position position;

    EXPECT_EQ(position.row, 0);
    EXPECT_EQ(position.column, 0);
}

TEST(PositionTest, StoresRowAndColumn) {
    const Position position{-2, 7};

    EXPECT_EQ(position.row, -2);
    EXPECT_EQ(position.column, 7);
}

TEST(PositionTest, ComparesBothCoordinates) {
    const Position position{3, 5};

    EXPECT_EQ(position, (Position{3, 5}));
    EXPECT_NE(position, (Position{4, 5}));
    EXPECT_NE(position, (Position{3, 6}));
}

TEST(PositionTest, CanBeUsedInUnorderedContainers) {
    std::unordered_set<Position, PositionHash> visited;

    visited.insert(Position{1, 2});
    visited.insert(Position{1, 2});
    visited.insert(Position{2, 1});

    EXPECT_EQ(visited.size(), 2U);
    EXPECT_EQ(visited.count(Position{1, 2}), 1U);
    EXPECT_EQ(visited.count(Position{9, 9}), 0U);
}

}  // namespace
}  // namespace dynamic_route
