# DynamicRoute

一个用于学习、复习和讲解网格路径规划的 C++17 项目。当前已完成核心数据模型、四种算法、动态修复、149 项正确性测试，以及静态和六场景动态评测。

## 从哪里开始

| 想做什么 | 阅读入口 |
| --- | --- |
| 配置环境、编译代码、运行程序 | [运行指南](docs/RUNNING.md) |
| 直观看到静态地图、障碍和三种算法的路径 | [静态路径图](docs/RUNNING.md#31-静态地图与路径图推荐) |
| 查看实际路径坐标、代价和统计 | [运行小示例](docs/RUNNING.md#3-运行小示例直接查看路径)及其[源码](docs/examples/basic_usage.cpp) |
| 运行全部测试或单个测试、查看日志 | [测试运行](docs/RUNNING.md#4-运行测试) |
| 运行评测、保存和筛选输出 | [Release 评测运行](docs/RUNNING.md#5-构建并运行-release-评测) |
| 理解字段、六个场景和计时范围 | [评测与结果解读](docs/BENCHMARKS.md) |
| 复习算法、接口和 D* Lite 的关键变量 | [算法与代码阅读](docs/ALGORITHMS.md) |
| VS Code 构建、调试与常见报错 | [VS Code 使用](docs/RUNNING.md#6-vs-code-中构建运行和调试)与[常见问题](docs/RUNNING.md#7-常见问题) |

## 当前功能

- `Position`、`GridMap`、`SearchResult`、`SearchStats`，地图与搜索状态分离。
- Dijkstra、A*、双向 A*、D* Lite，支持四/八邻接、禁止对角线穿过障碍拐角。
- 有限正地形权值（包括小于 1），每步费用为步长乘目标格权值，起点自身不收费。
- A* 和双向 A* 可选 Automatic、Euclidean、Zero 启发式。
- D* Lite 支持障碍变化、权值变化和起点移动，保留 g/rhs 及有效队列信息。
- GoogleTest 正确性测试、固定输入的 Release 性能评测与可运行的路径输出示例。
- 静态可视化示例：生成包含 Dijkstra、A*、双向 A* 路径图的独立 HTML，在浏览器中查看，无额外绘图库依赖。

默认四邻接。不可达时返回失败结果；非法端点抛异常。D* Lite 借用地图，更新必须经过规划器，地图生命周期要长于规划器；目标固定，当前未开放自定义 D* Lite 启发式。

## 快速运行

下面在 Windows PowerShell 中执行，工作目录为包含 `CMakeLists.txt` 的仓库根目录。请先安装 CMake、Ninja 及支持 C++17 的编译器，并加入 PATH；Windows 推荐统一使用 MSYS2 UCRT64 工具链。首次启用测试需下载 GoogleTest，详细前提和排错见运行指南。

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DDYNAMIC_ROUTE_BUILD_BENCHMARKS=ON -DDYNAMIC_ROUTE_BUILD_EXAMPLES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build
ctest --test-dir build --output-on-failure
.\build\examples\dynamic_route_static_visualization.exe ./build/static_paths.html
Start-Process ./build/static_paths.html
```

每条命令成功后再执行下一条。预期测试全部通过，示例生成并打开三种算法的静态路径图。需要具体坐标时仍可使用运行指南中的 basic_usage 示例。性能计时请使用独立的 Release 构建目录，图形展示与评测分开。

## 目录与构建目标

| 目录 | 内容 |
| --- | --- |
| `include/dynamic_route/` | 对外类型与算法接口。 |
| `src/` | 核心实现，不包含应用程序主函数。 |
| `tests/` | GoogleTest 正确性测试。 |
| `benchmarks/` | 静态初次规划、动态更新评测和共享校验。 |
| `examples/` | 独立静态路径可视化示例，通过 CMake 构建。 |
| `docs/` | 中文使用、算法和评测文档，以及独立编译的小示例。 |
| `build/` | 生成的 Debug 构建、测试程序和日志，不随仓库上传。 |
| `build/benchmark-release/` | 独立 Release 评测构建及手动保存的结果。 |

核心库目标是 `dynamic_route_core`，同一 CMake 工程中可通过 `DynamicRoute::core` 链接。测试目标是 `dynamic_route_tests`；评测目标是 `dynamic_route_benchmark`、`dynamic_route_dynamic_benchmark`，须显式启用 `DYNAMIC_ROUTE_BUILD_BENCHMARKS=ON`。`docs/examples/basic_usage.cpp` 按运行指南独立编译，不加入默认构建或测试。

图形示例目标为 `dynamic_route_static_visualization`，由 `DYNAMIC_ROUTE_BUILD_EXAMPLES=ON` 启用，默认关闭，上述命令已显式开启。它输出图文件，不是交互式 GUI 或性能评测。

## 完成范围与限制

本轮基础开发、评测和文档整理已完成。项目侧重单人可读、可解释的独立实现，不保证某一种算法在所有输入上最快。六个评测场景都使用 Four；Eight 及混合动态行为由正确性测试覆盖，不意味着已有全面性能评测。

当前没有 GUI、地图文件加载、交互式编辑或自动化性能排名，也没有引入 ROS、CUDA、网络或其他规划算法。后续扩展应作为新的小阶段推进。`path_planning` 是外部参考项目，不包含在本仓库中，也不参与构建。

详细验证范围见 [运行指南末尾](docs/RUNNING.md#验证记录)。本仓库仅包含源码和项目文档，生成文件及个人编辑器配置由 `.gitignore` 排除。
