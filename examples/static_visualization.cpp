#include "dynamic_route/a_star.hpp"
#include "dynamic_route/bidirectional_a_star.hpp"
#include "dynamic_route/dijkstra.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

using namespace dynamic_route;

constexpr int cell_size = 24;
constexpr int margin = 24;

GridMap make_demo_map() {
    GridMap map{15, 23}; // All terrain weights are 1; geometry is the focus here.
    for (int row = 0; row < 15; ++row) {
        if (row != 3 && row != 11) {
            map.set_blocked(Position{row, 6}, true);
        }
        if (row != 7) {
            map.set_blocked(Position{row, 12}, true);
        }
        if (row != 3) {
            map.set_blocked(Position{row, 18}, true);
        }
    }
    for (int column = 1; column < 5; ++column) {
        map.set_blocked(Position{5, column}, true);
    }
    for (int column = 14; column < 18; ++column) {
        map.set_blocked(Position{10, column}, true);
    }
    return map;
}

void validate(const GridMap& map, const Position start, const Position goal,
              const SearchResult& result, const double reference_cost) {
    if (!result.found || result.path.empty() || result.path.front() != start ||
        result.path.back() != goal) {
        throw std::runtime_error("Visualization requires a path with valid endpoints");
    }
    double cost = 0.0;
    for (std::size_t index = 1; index < result.path.size(); ++index) {
        cost += movement_cost(map, result.path[index - 1], result.path[index]);
    }
    const double tolerance = 1e-9 * std::max(1.0, std::abs(reference_cost));
    if (!std::isfinite(cost) || !std::isfinite(result.total_cost) ||
        std::abs(cost - result.total_cost) > tolerance ||
        std::abs(cost - reference_cost) > tolerance) {
        throw std::runtime_error("Visualization path cost does not match Dijkstra");
    }
}

int center(const int coordinate) {
    return margin + coordinate * cell_size + cell_size / 2;
}

void draw_endpoint(std::ostream& out, const Position position, const char label,
                   const char* color) {
    out << "<circle cx=\"" << center(position.column) << "\" cy=\""
        << center(position.row) << "\" r=\"10\" fill=\"" << color << "\"/>\n"
        << "<text x=\"" << center(position.column) << "\" y=\""
        << center(position.row) + 4
        << "\" text-anchor=\"middle\" fill=\"white\" font-size=\"12\" font-weight=\"700\">"
        << label << "</text>\n";
}

void draw_map(std::ostream& out, const char* name, const char* color,
              const GridMap& map, const Position start, const Position goal,
              const SearchResult& result) {
    out << "<section><h2 style=\"color:" << color << "\">" << name << "</h2>\n"
        << "<p>总代价 <strong>" << result.total_cost << "</strong> · 路径步数 "
        << result.path.size() - 1 << "<br>扩展次数 " << result.stats.expanded_nodes
        << " · 入队次数 " << result.stats.frontier_pushes << "</p>\n"
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" role=\"img\" viewBox=\"0 0 "
        << map.column_count() * cell_size + margin * 2 << ' '
        << map.row_count() * cell_size + margin * 2 << "\"><title>" << name
        << "：静态地图与最终路径</title>\n";
    for (int row = 0; row < map.row_count(); ++row) {
        for (int column = 0; column < map.column_count(); ++column) {
            const bool blocked = map.is_blocked(Position{row, column});
            out << "<rect x=\"" << margin + column * cell_size << "\" y=\""
                << margin + row * cell_size << "\" width=\"" << cell_size
                << "\" height=\"" << cell_size << "\" fill=\""
                << (blocked ? "#334155" : "#f8fafc")
                << "\" stroke=\"#cbd5e1\" stroke-width=\"0.6\"><title>("
                << row << ',' << column << ") " << (blocked ? "blocked" : "free")
                << "</title></rect>\n";
        }
        out << "<text x=\"16\" y=\"" << center(row) + 3
            << "\" text-anchor=\"end\" font-size=\"9\" fill=\"#64748b\">"
            << row << "</text>\n";
    }
    for (int column = 0; column < map.column_count(); ++column) {
        out << "<text x=\"" << center(column)
            << "\" y=\"16\" text-anchor=\"middle\" font-size=\"9\" fill=\"#64748b\">"
            << column << "</text>\n";
    }
    out << "<polyline fill=\"none\" stroke=\"" << color
        << "\" stroke-width=\"4\" stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
    for (const Position position : result.path) {
        out << center(position.column) << ',' << center(position.row) << ' ';
    }
    out << "\"/>\n";
    draw_endpoint(out, start, 'S', "#059669");
    draw_endpoint(out, goal, 'G', "#dc2626");
    out << "</svg></section>\n";
}

void write_header(std::ostream& out) {
    out << R"(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DynamicRoute · 静态路径规划</title>
<style>
*{box-sizing:border-box}body{margin:0;padding:32px;background:#f1f5f9;color:#0f172a;font:15px/1.7 system-ui,"Microsoft YaHei",sans-serif}
header,main,footer{max-width:1600px;margin:auto}h1{margin:0;font-size:28px}header p{color:#475569;margin:8px 0 20px}
.legend{display:flex;gap:16px;flex-wrap:wrap;margin-bottom:24px}.legend span{display:flex;align-items:center;gap:6px}
.swatch{width:16px;height:16px;display:inline-block;border:1px solid #cbd5e1;border-radius:3px}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,380px),1fr));gap:20px}
section{min-width:0;background:white;border:1px solid #e2e8f0;border-radius:12px;padding:20px}h2{margin:0;font-size:20px}
section p{color:#475569;margin:8px 0 12px}svg{width:100%;height:auto;display:block}footer{color:#64748b;margin-top:22px}
</style></head><body><header><h1>静态路径规划</h1>
<p>同一张 15 × 23 地图 · 四邻接 · 所有地形权值为 1 · 起点 (12,1) → 目标 (2,21)</p>
<div class="legend"><span><i class="swatch" style="background:#334155"></i>障碍</span>
<span><i class="swatch" style="background:#f8fafc"></i>可通行</span>
<span><i class="swatch" style="background:#059669"></i>S 起点</span>
<span><i class="swatch" style="background:#dc2626"></i>G 目标</span><span>彩色折线：最终规划路径</span></div></header><main>
)";
}

}  // namespace

int main(const int argc, char* argv[]) {
    try {
        if (argc > 2) {
            throw std::invalid_argument("Usage: dynamic_route_static_visualization [output.html]");
        }
        const GridMap map = make_demo_map();
        const Position start{12, 1};
        const Position goal{2, 21};
        const std::array<SearchResult, 3> results{
            dijkstra(map, start, goal), a_star(map, start, goal),
            bidirectional_a_star(map, start, goal)};
        constexpr std::array<const char*, 3> names{"Dijkstra", "A*", "Bidirectional A*"};
        constexpr std::array<const char*, 3> colors{"#2563eb", "#7c3aed", "#d97706"};
        for (const SearchResult& result : results) {
            validate(map, start, goal, result, results.front().total_cost);
        }

        const std::filesystem::path output = argc == 2 ? argv[1] : "static_paths.html";
        if (!output.parent_path().empty()) {
            std::filesystem::create_directories(output.parent_path());
        }
        std::ofstream file;
        file.exceptions(std::ios::failbit | std::ios::badbit);
        file.open(output, std::ios::binary | std::ios::trunc);
        file << std::fixed << std::setprecision(2);
        write_header(file);
        for (std::size_t index = 0; index < results.size(); ++index) {
            draw_map(file, names[index], colors[index], map, start, goal, results[index]);
            std::cout << names[index] << ": cost=" << results[index].total_cost
                      << ", expanded=" << results[index].stats.expanded_nodes << '\n';
        }
        file << "</main><footer>坐标从 0 开始。路线经过格子中心；不同最优路径可以有相同代价。"
                "这里显示最终路径，不显示搜索过程动画；计时评测请使用 benchmark 程序。"
                "</footer></body></html>\n";
        file.close();
        std::cout << "Saved: " << std::filesystem::absolute(output).string() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Visualization failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
