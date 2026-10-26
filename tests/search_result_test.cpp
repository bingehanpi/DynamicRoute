#include "dynamic_route/search_result.hpp"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace dynamic_route {
namespace {

TEST(SearchStatsTest, CountersStartAtZero) {
    const SearchStats stats;

    EXPECT_EQ(stats.expanded_nodes, 0U);
    EXPECT_EQ(stats.frontier_pushes, 0U);
}

TEST(SearchResultTest, DefaultsToNoPathWithInfiniteCost) {
    const SearchResult result;

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.total_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(result.stats.expanded_nodes, 0U);
    EXPECT_EQ(result.stats.frontier_pushes, 0U);
}

TEST(SearchResultTest, CanRepresentAPathWithCostAndStatistics) {
    // Construct sample output; no search algorithm is executed here.
    const SearchResult result{
        true,
        {{0, 0}, {0, 1}, {1, 1}},
        3.5,
        {2, 4}
    };
    const std::vector<Position> expected_path{{0, 0}, {0, 1}, {1, 1}};

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path, expected_path);
    EXPECT_DOUBLE_EQ(result.total_cost, 3.5);
    EXPECT_EQ(result.stats.expanded_nodes, 2U);
    EXPECT_EQ(result.stats.frontier_pushes, 4U);
}

TEST(SearchResultTest, CanRepresentStartEqualToGoal) {
    const Position start{1, 2};
    const SearchResult result{true, {start}, 0.0, {}};

    EXPECT_TRUE(result.found);
    ASSERT_EQ(result.path.size(), 1U);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

}  // namespace
}  // namespace dynamic_route
