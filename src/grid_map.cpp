#include "dynamic_route/grid_map.hpp"

#include <cmath>
#include <stdexcept>

namespace dynamic_route {

GridMap::GridMap(const int row_count, const int column_count)
    : row_count_(row_count), column_count_(column_count) {
    if (row_count <= 0 || column_count <= 0) {
        throw std::invalid_argument("GridMap dimensions must be positive");
    }

    const auto rows = static_cast<std::size_t>(row_count);
    const auto columns = static_cast<std::size_t>(column_count);
    // Check before multiplying so the cell count cannot overflow.
    if (rows > blocked_.max_size() / columns ||
        rows > terrain_weights_.max_size() / columns) {
        throw std::length_error("GridMap dimensions exceed storage capacity");
    }
    blocked_.assign(rows * columns, 0);
    terrain_weights_.assign(rows * columns, 1.0);
}

int GridMap::row_count() const noexcept {
    return row_count_;
}

int GridMap::column_count() const noexcept {
    return column_count_;
}

bool GridMap::contains(const Position position) const noexcept {
    return position.row >= 0 && position.row < row_count_ && position.column >= 0 &&
           position.column < column_count_;
}

bool GridMap::is_blocked(const Position position) const {
    return blocked_[index_of(position)] != 0;
}

void GridMap::set_blocked(const Position position, const bool blocked) {
    blocked_[index_of(position)] = blocked ? 1 : 0;
}

double GridMap::terrain_weight(const Position position) const {
    return terrain_weights_[index_of(position)];
}

void GridMap::set_terrain_weight(const Position position, const double weight) {
    const std::size_t index = index_of(position);
    if (!std::isfinite(weight) || weight <= 0.0) {
        throw std::invalid_argument("Terrain weight must be finite and positive");
    }

    terrain_weights_[index] = weight;
}

std::size_t GridMap::index_of(const Position position) const {
    if (!contains(position)) {
        throw std::out_of_range("Position is outside GridMap bounds");
    }

    // Row-major storage: each complete row occupies column_count_ cells.
    return static_cast<std::size_t>(position.row) *
               static_cast<std::size_t>(column_count_) +
           static_cast<std::size_t>(position.column);
}

}  // namespace dynamic_route
