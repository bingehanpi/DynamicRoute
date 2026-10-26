#pragma once

#include <cstddef>
#include <functional>

namespace dynamic_route {

struct Position {
    int row{0};
    int column{0};

    [[nodiscard]] constexpr bool operator==(const Position& other) const noexcept {
        return row == other.row && column == other.column;
    }

    [[nodiscard]] constexpr bool operator!=(const Position& other) const noexcept {
        return !(*this == other);
    }
};

struct PositionHash {
    [[nodiscard]] std::size_t operator()(const Position& position) const noexcept {
        constexpr std::size_t hash_mixing_constant = 0x9e3779b9U;
        const std::size_t row_hash = std::hash<int>{}(position.row);
        const std::size_t column_hash = std::hash<int>{}(position.column);

        // Mix both component hashes so coordinate order affects the result.
        return row_hash ^ (column_hash + hash_mixing_constant + (row_hash << 6U) +
                           (row_hash >> 2U));
    }
};

}  // namespace dynamic_route
