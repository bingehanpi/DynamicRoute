#pragma once

#include "dynamic_route/position.hpp"

#include <cstddef>
#include <vector>

namespace dynamic_route {

class GridMap {
public:
    GridMap(int row_count, int column_count);

    [[nodiscard]] int row_count() const noexcept;
    [[nodiscard]] int column_count() const noexcept;
    [[nodiscard]] bool contains(Position position) const noexcept;

    // Both operations throw std::out_of_range for positions outside the map.
    [[nodiscard]] bool is_blocked(Position position) const;
    void set_blocked(Position position, bool blocked);

    // Weights default to 1.0 and are independent of obstacle state.
    // Both operations throw std::out_of_range for positions outside the map.
    [[nodiscard]] double terrain_weight(Position position) const;
    // Throws std::invalid_argument unless weight is finite and positive.
    void set_terrain_weight(Position position, double weight);

private:
    [[nodiscard]] std::size_t index_of(Position position) const;

    int row_count_;
    int column_count_;
    std::vector<unsigned char> blocked_;
    std::vector<double> terrain_weights_;
};

}  // namespace dynamic_route
