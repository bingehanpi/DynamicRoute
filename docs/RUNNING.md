# 构建、运行、测试与结果查看

这份指南使用 Windows + PowerShell + MSYS2 UCRT64 工具链。所有命令都在包含 `CMakeLists.txt` 的仓库根目录执行，源码和输出位置使用相对路径。

## 1. 进入正确目录并准备环境

```powershell
Get-Command cmake, ctest, g++, ninja
cmake --version
g++ --version
ninja --version
```

需要 CMake 3.20 及以上、Ninja 和支持 C++17 的编译器；调试时还需要 GDB。Windows 下应将所选 UCRT64 工具链的 bin 目录及 CMake 加入 PATH，然后重新打开终端。工具安装位置由使用者自行选择，仓库不绑定个人安装路径。

通过 `Get-Command` 确认实际使用的工具来自预期安装，编译核心库和示例时使用同一套编译器及运行库。如果刚克隆仓库且终端在上级目录，可先执行 `Set-Location ./DynamicRoute`。带空格的路径应加引号。

首次启用测试时，CMake 会下载 GoogleTest v1.17.0，需要网络。已有 `build/_deps` 缓存时通常会复用。以下命令验证过已有依赖缓存的构建流程，首次联网下载是否可用取决于当时网络。

## 2. 配置并构建 Debug 版本

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DDYNAMIC_ROUTE_BUILD_BENCHMARKS=ON -DDYNAMIC_ROUTE_BUILD_EXAMPLES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build
```

每条命令执行成功后再运行下一条。PowerShell 中可用 `$LASTEXITCODE` 查看刚执行的外部程序退出码，`0` 表示成功。

| 配置项 | 用途 |
| --- | --- |
| `CMAKE_BUILD_TYPE=Debug` | 保留调试信息，方便断点查看变量。 |
| `BUILD_TESTING=ON` | 构建 GoogleTest 测试程序；默认开启。 |
| `DYNAMIC_ROUTE_BUILD_BENCHMARKS=ON` | 构建两个评测程序；项目默认关闭，本文显式开启。 |
| `DYNAMIC_ROUTE_BUILD_EXAMPLES=ON` | 构建静态路径图形示例；默认关闭，上述命令显式开启。 |
| `CMAKE_EXPORT_COMPILE_COMMANDS=ON` | 生成编译数据库，供编辑器获取编译参数。 |

修改普通 `.cpp` 后一般只需再次执行 `cmake --build build`。首次配置、修改构建选项或添加构建目标时，重新运行配置命令。上述 Debug 配置也包含评测文件，让 IntelliSense 能获取它们的头文件路径。

## 3. 运行小示例，直接查看路径

### 3.1 静态地图与路径图（推荐）

先完成上一节的配置和构建，或在当前已配置的 VS Code 工作区按 Ctrl+Shift+B。然后在项目根目录的集成终端执行：

```powershell
.\build\examples\dynamic_route_static_visualization.exe ./build/static_paths.html
Start-Process ./build/static_paths.html
```

第一条运行 C++ 算法并生成图页，第二条通过系统默认浏览器打开。也可以在文件资源管理器中双击 `build/static_paths.html`。图页是自包含 HTML，内嵌 SVG 矢量图，无需网络、Python 或额外绘图库。

图中三种算法使用同一张 15×23 的固定地图，四邻接、全图权值为 1；起点 `(12,1)`，目标 `(2,21)`。每个面板包含地图、最终路线及代价/步数/扩展/入队次数。深色块是障碍，绿色 S 是起点，红色 G 是目标，彩色折线是规划路径；行列编号从 0 开始。三者最优代价均为 30，路径形状可能因并列最优选择不同而不同。

程序会打印生成文件的绝对路径。可给第一个参数指定其他输出位置，例如 `./build/my-results/paths.html`，父目录不存在时自动创建；省略参数时写入当前工作目录的 `static_paths.html`。再次运行会覆盖同名文件，浏览器已打开时需刷新页面。CMake Tools 不带参数运行这个目标时，应按终端打印的位置找到图页。

修改示例地图可查看 `examples/static_visualization.cpp` 的 `make_demo_map()`，然后重新构建、运行。当前展示固定地图上的最终路径，不包含搜索过程动画或动态地图播放。它与 48×64 的 benchmark 地图不同，绘图不加入原评测计时。

### 3.2 具体坐标与 D* Lite 接口示例

核心代码是库，本身没有交互式 `main()`。文档附带了可运行的 [basic_usage.cpp](examples/basic_usage.cpp)，用于观察四种算法和 D* Lite 动态更新的结果。

先完成上一节的构建，再执行：

```powershell
g++ -std=c++17 -Wall -Wextra -Wpedantic -g -O0 -I ./include ./docs/examples/basic_usage.cpp ./build/libdynamic_route_core.a -o ./build/basic_usage.exe
.\build\basic_usage.exe
```

这个命令适用于本文的 UCRT64 + Ninja 构建。`-I ./include` 指定头文件目录，`libdynamic_route_core.a` 提供算法实现，位置在示例源文件之后。编译器须与构建核心库时一致。示例不加入 CMake 默认目标或 CTest；修改示例或重新构建核心库后，重新执行这条编译命令。

示例地图为 5 行 7 列，初始全通行、权值全为 1，起点 `(2,0)`、目标 `(2,6)`。每组输出包含：

```text
Dijkstra: found=true, cost=6.00, expanded=..., pushes=...
  path: (2,0) (2,1) (2,2) (2,3) (2,4) (2,5) (2,6)
```

省略号表示这里未固定展示的统计值，实际程序会输出数字。

| 输出标签 | 操作 | 预期代价 |
| --- | --- | --- |
| Dijkstra / AStar / BidirectionalAStar | 初始地图分别搜索。 | 6.00 |
| DStarLite initial | 首次规划。 | 6.00 |
| DStarLite unchanged | 不修改地图，再次规划。 | 6.00，expanded 和 pushes 都为 0 |
| DStarLite blocked | 阻塞 `(2,3)`，修复并绕行。 | 8.00 |
| DStarLite restored | 恢复 `(2,3)` 通行。 | 6.00 |
| DStarLite cheaper terrain | 将 `(2,3)` 权值改为 0.5。 | 5.50 |
| DStarLite moved start | 起点移动到 `(2,1)`，保留权值变化。 | 4.50 |

坐标格式为 `(row,column)`，从 0 开始。`path` 包含起点与目标；相同最优代价可能对应不同路径。具体字段见 [算法与接口说明](ALGORITHMS.md)。

需要留存完整路径输出时：

```powershell
.\build\basic_usage.exe | Tee-Object -FilePath ./build/basic-usage-results.txt
Get-Content ./build/basic-usage-results.txt
```

## 4. 运行测试

运行全部测试：

```powershell
ctest --test-dir build --output-on-failure
```

当前版本应报告 **149 项测试全部通过**。`--output-on-failure` 在失败时显示详细输出。测试数是本次整理时的状态，后续增加测试后会改变。

只列出测试，不执行：

```powershell
ctest --test-dir build -N
```

只运行 D* Lite 测试：

```powershell
ctest --test-dir build -R '^DStarLiteTest\.' --output-on-failure
```

当前此筛选应运行 44 项测试。

只运行一个测试，并显示它的完整输出：

```powershell
ctest --test-dir build -R '^DStarLiteTest.MovingAlongKnownPathReusesGoalDistances$' -V
```

`-R` 使用正则表达式。也可以直接使用 GoogleTest 程序，其过滤语法不同：

```powershell
.\build\tests\dynamic_route_tests.exe --gtest_list_tests
.\build\tests\dynamic_route_tests.exe '--gtest_filter=DStarLiteTest.MovingAlongKnownPathReusesGoalDistances'
```

GoogleTest 用 `套件名.测试名` 和通配符过滤；CTest 的测试名来自同一批 GoogleTest 测试。测试通常输出通过/失败信息，不会逐条打印路径。想看路径请运行小示例，或调试测试中的 `SearchResult`。

保存测试日志并保留退出码：

```powershell
ctest --test-dir build --output-on-failure 2>&1 | Tee-Object -FilePath ./build/test-results.txt
$testExitCode = $LASTEXITCODE
Get-Content ./build/test-results.txt
$testExitCode
```

CTest 最近一次执行的内部日志也可在 `build/Testing/Temporary/LastTest.log` 查看。

## 5. 构建并运行 Release 评测

计时使用独立 Release 构建，便于与 Debug 调试目录区分：

```powershell
cmake -S . -B build/benchmark-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DDYNAMIC_ROUTE_BUILD_BENCHMARKS=ON
cmake --build build/benchmark-release
.\build\benchmark-release\benchmarks\dynamic_route_benchmark.exe
.\build\benchmark-release\benchmarks\dynamic_route_dynamic_benchmark.exe
```

第一个程序评测四种算法的初次规划，第二个程序运行六个动态场景。它们均无命令行参数，场景与重复次数在源码中固定；修改后须重新构建。Release 配置关闭测试，不下载 GoogleTest，也不包含测试程序。正确性测试仍在上一节的 Debug 目录运行。

程序把结果输出到终端，**不会自动生成结果文件**。下列命令同时显示并保存结果，重复运行会覆盖同名文件：

```powershell
.\build\benchmark-release\benchmarks\dynamic_route_benchmark.exe | Tee-Object -FilePath ./build/benchmark-release/static-results.txt
$staticExitCode = $LASTEXITCODE
.\build\benchmark-release\benchmarks\dynamic_route_dynamic_benchmark.exe | Tee-Object -FilePath ./build/benchmark-release/dynamic-results.txt
$dynamicExitCode = $LASTEXITCODE
$staticExitCode
$dynamicExitCode
Get-Content ./build/benchmark-release/dynamic-results.txt
```

正常情况下两个退出码均为 `0`；路径校验失败时程序会报错并返回非零值。结果文件保留标准输出；如需同时保存错误信息，可在管道之前加 `2>&1`。

只查看起点移动场景的四行结果：

```powershell
Select-String -LiteralPath ./build/benchmark-release/dynamic-results.txt -Pattern '^open_moving_start,'
```

动态输出每个场景都有注释和表头，不能把整个文件直接当成只有一个表头的 CSV。需要在 PowerShell 中筛选、排序时：

```powershell
$dynamicRows = Get-Content ./build/benchmark-release/dynamic-results.txt | Where-Object { $_ -notmatch '^#' -and $_ -notmatch '^scenario,' -and $_.Trim() -ne '' }
$dynamicResults = $dynamicRows | ConvertFrom-Csv -Header scenario,algorithm,initial_us,first_update_us,mean_later_update_us,mean_update_us,total_us,mean_expanded_nodes,mean_frontier_pushes
$dynamicResults | Where-Object scenario -eq 'open_moving_start' | Format-Table algorithm,mean_update_us,total_us,mean_expanded_nodes
```

这些日志和程序都在 `build` 下，属于生成物，不包含在源码仓库中。清理构建目录时不会自动保留，需要长期保存某次实验时请另行备份。

各列意义、场景输入和计时范围见 [评测与结果解读](BENCHMARKS.md)。结果文件只代表保存时的那次运行，不是程序自动维护的报告。

## 6. VS Code 中构建、运行和调试

1. 打开仓库根目录，资源管理器根部应能看到 `CMakeLists.txt`、`include`、`src`、`tests` 和 `benchmarks`。
2. 安装 Microsoft C/C++ 和 CMake Tools 扩展。个人 `.vscode` 配置不随仓库提交；工具通过 PATH 或自己的本地设置选择。命令行指南可独立使用。
3. 先按第 2 节完成配置，或为 CMake Tools 设置相同的生成器、构建目录和开关后执行 **CMake: Configure**。若要让 Ctrl+Shift+B 构建项目，在本地创建下面的 `.vscode/tasks.json`，默认任务将执行 `cmake --build build`。
4. 最稳妥的运行方法是在集成 PowerShell 终端执行本指南中的程序路径。使用 CMake Tools 时，在 CMake 项目目标列表选择 `dynamic_route_static_visualization`、`dynamic_route_benchmark`、`dynamic_route_dynamic_benchmark` 或 `dynamic_route_tests`，再使用其运行/调试入口。不同扩展版本入口位置可能不同。
5. 不选择 C/C++ 的“生成和调试活动文件”。普通项目 `.cpp` 依赖其他源文件和项目头文件目录，应通过 CMake 构建。
6. CMake Tools 的构建目录应指向 `build` 的 Debug 版本。查看性能数据时，在终端运行 `build/benchmark-release` 中的 Release 程序。

可在本地使用的 `.vscode/tasks.json`（要求工具已加入 PATH）：

```json
{
    "version": "2.0.0",
    "tasks": [{
        "label": "CMake: 构建项目",
        "type": "process",
        "command": "cmake",
        "args": ["--build", "${workspaceFolder}/build"],
        "options": {"cwd": "${workspaceFolder}"},
        "problemMatcher": ["$gcc"],
        "group": {"kind": "build", "isDefault": true}
    }]
}
```

CMake Tools 可在本地 `.vscode/settings.json` 中设置 `cmake.generator` 为 `Ninja`、`cmake.buildDirectory` 为 `${workspaceFolder}/build`，并在 `cmake.configureSettings` 中开启本文所需开关。C/C++ 的 `C_Cpp.default.compileCommands` 可设为 `${workspaceFolder}/build/compile_commands.json`，语言标准设为 `c++17`。

如果 Ctrl+Shift+B 又输出 `g++.exe ... ${file}` 这类单文件编译命令，检查 `.vscode/tasks.json`：本项目应由 `CMake: 构建项目` 的 `group.isDefault=true` 作为默认构建任务。命令面板执行 **Tasks: Configure Default Build Task**（配置默认生成任务），选择 **CMake: 构建项目**。若仍使用旧任务，先执行 **Developer: Reload Window**，再重新选择默认任务。C/C++ 的活动文件运行/调试入口可能另行生成任务；之后不要将它选为此项目的默认构建任务。

查看路径变量：在 `benchmarks/static_benchmark.cpp` 的 `validate(...)` 调用处设置断点，调试 `dynamic_route_benchmark`，停住后查看 `current.found`、`current.total_cost`、`current.path` 和 `current.stats`。若想观察 D* Lite 更新，可在动态评测的更新结果校验处断点，查看 `result`、`starts[step + 1]` 和 `step`。

示例程序独立编译后，也可在终端用 GDB 查看：

```powershell
gdb ./build/basic_usage.exe
```

进入 GDB 后输入 `break print_result`、`run`、`print result.total_cost`、`print result.path`，用 `continue` 继续、`quit` 退出。

## 7. 常见问题

| 现象 | 排查与处理 |
| --- | --- |
| Ctrl+Shift+B 仍使用单文件 `g++` 命令 | 查看默认任务是否又变成“生成活动文件”，按上一节恢复 CMake 默认任务。修改编译器的 IntelliSense 设置不会自动替换这条任务命令。 |
| `dynamic_route/a_star.hpp: No such file or directory` | 检查是否误用了活动文件编译任务。项目通过 CMake 构建；文档示例使用上面的完整编译命令，包含 `-I` 和核心库。 |
| `std::optional` 附近显示“缺少显式类型” | 确认 C++17、同一套编译器、仓库根目录工作区及 `build/compile_commands.json`。重新 Configure 并确认评测开关为 ON，然后重载窗口；仍存在时查看 C/C++ 诊断日志中的实际配置。 |
| `undefined reference` | 检查是否只编译了一个 `.cpp`，却没有链接核心库。用 CMake 构建项目；示例按完整命令链接。 |
| 找不到评测 `.exe` 或构建目标 | 重新配置 `DYNAMIC_ROUTE_BUILD_BENCHMARKS=ON`，确认构建成功，并区分 Debug 与 Release 路径。 |
| 找不到静态图形示例 | 重新配置 `DYNAMIC_ROUTE_BUILD_EXAMPLES=ON` 并构建；程序在 `build/examples` 中。 |
| CTest 提示没有测试 | 在 `build` 中配置 `BUILD_TESTING=ON` 并构建；本文的 Release 评测目录特意关闭了测试。 |
| 程序无法启动、缺少 DLL 或编译器异常退出 | 检查 PATH，确保构建时使用的 UCRT64 运行库及工具可用。 |
| 配置时报 GoogleTest 下载失败 | 检查能否访问配置中的下载地址；先恢复网络后重新 Configure。只运行评测可用 `BUILD_TESTING=OFF`。不要把关闭测试误当成测试通过。 |
| CMake 报生成器不一致 | 使用原生成器，或换一个新构建目录重新配置。切换编译器时也建议使用新目录；同步修改后续命令中的构建目录。 |
| 相同输入的耗时不同 | 系统负载、编译优化、缓存和执行顺序都会影响计时。确认使用 Release，并结合工作量统计分析。 |

## 验证记录

验证范围包括 Debug 配置/构建、149 项测试、CTest 列表与过滤（D* Lite 44 项及单项）、GoogleTest 直接运行、文档示例编译/运行、GDB 断点查看代价和路径、Release 配置/构建、两个评测及结果保存/筛选。

GoogleTest 验证使用已有缓存，首次联网下载取决于网络环境；VS Code 图形界面操作未由自动化直接验证。文档链接及代码块配对已检查。

静态可视化已验证配置/构建、三条导出路线的合法性与最优代价、浏览器渲染、默认输出、带空格的嵌套输出路径、非法参数及写入失败的退出状态。生成的页面不依赖验证时使用的浏览器工具。
