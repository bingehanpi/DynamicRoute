#pragma once

#include "dynamic_route/movement.hpp"
#include "dynamic_route/search_result.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace dynamic_route::benchmark {

inline constexpr int rows = 48;
inline constexpr int columns = 64;
inline constexpr unsigned int seed = 20261024U;
inline constexpr int repetitions = 20;

inline GridMap make_map() {
    GridMap map{rows, columns};
    std::mt19937 random{seed};
    constexpr double weights[] = {0.25, 0.5, 1.0, 2.0, 5.0};
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const Position cell{row, column};
            map.set_terrain_weight(cell, weights[random() % 5U]);
            const bool blocked = random() % 5U == 0U;
            // Keep a corridor down the first column and across the last row.
            map.set_blocked(cell, blocked && column != 0 && row != rows - 1);
        }
    }
    return map;
}

inline bool costs_match(const double actual, const double expected) {
    return std::isfinite(actual) &&
           std::abs(actual - expected) <= 1e-9 * std::max(1.0, std::abs(expected));
}

inline void validate(const char* name, const GridMap& map, const Position start,
                     const Position goal, const SearchResult& result,
                     const double reference_cost) {
    const auto fail = [name]() {
        throw std::runtime_error(std::string{name} + " returned an invalid or nonoptimal path");
    };
    if (reference_cost == std::numeric_limits<double>::infinity()) {
        if (result.found || !result.path.empty() || result.total_cost != reference_cost) {
            fail();
        }
        return;
    }
    if (!result.found || result.path.empty() || result.path.front() != start ||
        result.path.back() != goal || !costs_match(result.total_cost, reference_cost)) {
        fail();
    }
    double path_cost = 0.0;
    for (std::size_t step = 1; step < result.path.size(); ++step) {
        // Also checks legal four-connected moves and obstacle avoidance.
        path_cost += movement_cost(map, result.path[step - 1], result.path[step]);
    }
    if (!costs_match(path_cost, result.total_cost)) {
        fail();
    }
}

}  // namespace dynamic_route::benchmark
