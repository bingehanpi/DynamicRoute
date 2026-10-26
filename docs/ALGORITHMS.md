# 算法、接口与代码阅读

运行命令见 [运行指南](RUNNING.md)，评测输入和输出见 [评测说明](BENCHMARKS.md)。本文描述当前项目的实现与约定。

想把搜索结果直接显示在地图上，可运行 [静态路径图示例](RUNNING.md#31-静态地图与路径图推荐)。它独立调用现有算法并绘制最终路径，核心搜索代码不负责绘图。

## 1. 数据与移动规则

| 类型/文件 | 职责 |
| --- | --- |
| `include/dynamic_route/position.hpp` | `Position{row,column}`，从 0 开始的网格坐标。 |
| `include/dynamic_route/grid_map.hpp` | `GridMap` 保存尺寸、障碍和地形权值，不保存搜索状态。 |
| `include/dynamic_route/movement.hpp` | 四/八邻接、合法移动、对角线拐角检查和边代价。 |
| `include/dynamic_route/heuristic.hpp` | 共享启发式距离与静态算法使用的最小通行权值扫描。 |
| `include/dynamic_route/search_result.hpp` | 路径、总代价及本次搜索工作量统计。 |

`GridMap{rows,columns}` 要求行列数为正；初始全部通行、权值均为 1。尺寸非正抛出 `std::invalid_argument`，超过容器容量的尺寸抛出 `std::length_error`。障碍状态与权值独立，权值必须是有限正数，允许小于 1。

默认 `Connectivity::Four`，可选 `Connectivity::Eight`。每一步的代价为：

```text
c(u,v) = 步长 × terrain_weight(v)
直线步长 = 1
对角线步长 = sqrt(2)
```

起点自身不收费，路径代价是沿起点到目标方向逐边相加。权值不同会使 `c(u,v)` 与 `c(v,u)` 不同，即使几何邻接是对称的。反向搜索仍须使用原正向边的费用。

对角线只有在目标格及两侧相邻格都通行时才允许，不能穿过障碍拐角。因此八邻接地图中的侧边格变化，也可能改变另外两个格子之间的对角边。

## 2. 搜索结果与错误约定

| 字段 | 含义 |
| --- | --- |
| `found` | 是否找到路径。 |
| `path` | 成功时按起点到目标排列，包含两个端点。 |
| `total_cost` | 返回路径的正向总代价；失败时为正无穷。 |
| `stats.expanded_nodes` | 本次节点处理/扩展次数，可能重复处理同一格。 |
| `stats.frontier_pushes` | 本次实际入队次数，包含初始化、重复记录或重新入队。 |

端点越界抛出 `std::out_of_range`；先检查两个端点的边界，再检查障碍。端点位于障碍上抛出 `std::invalid_argument`，即使起终点相同也不例外。合法但不可达时不抛异常，返回 `found=false`、空路径、无穷代价，并保留已进行的搜索统计。

合法的起点等于目标时，返回单点路径、代价 0、统计 0。统计只描述本次搜索工作，不包含全部运行成本；例如扫描权值、部分状态遍历和路径提取不能直接从扩展数读出。

## 3. 静态算法接口

以下代码片段假设已创建 `map`、`start` 和 `goal`；可运行的完整示例见 [basic_usage.cpp](examples/basic_usage.cpp)。

```cpp
#include "dynamic_route/dijkstra.hpp"
#include "dynamic_route/a_star.hpp"
#include "dynamic_route/bidirectional_a_star.hpp"

auto d = dynamic_route::dijkstra(map, start, goal);
auto a = dynamic_route::a_star(map, start, goal);
auto b = dynamic_route::bidirectional_a_star(map, start, goal);
```

| 算法 | 状态与搜索方式 | 主要用途 |
| --- | --- | --- |
| Dijkstra | 按实际累计代价搜索，每次调用创建新状态。 | 基础实现及最优代价参照。 |
| A* | 按 `f=g+h` 搜索，每次调用创建新状态。 | 用目标方向的下界减少搜索范围。 |
| 双向 A* | 两端交替搜索，分别维护队列、距离和父节点。 | 学习双向搜索、反向边费用及停止条件。 |

三个函数的第四个参数都可设置邻接方式；A* 和双向 A* 的第五个参数可选启发式：

```cpp
auto result = dynamic_route::a_star(
    map, start, goal, dynamic_route::Connectivity::Eight,
    dynamic_route::Heuristic::Euclidean);
```

| `Heuristic` | 未缩放的距离 |
| --- | --- |
| `Automatic`（默认） | Four 用 Manhattan：`dr+dc`；Eight 用 Octile：`sqrt(2)*min(dr,dc)+abs(dr-dc)`。 |
| `Euclidean` | 欧氏直线距离。 |
| `Zero` | 恒为 0，按实际代价排列队列。 |

非零距离乘以本次地图最小的**通行格权值**，保持权值小于 1 时的下界有效。每次调用重新扫描；`Zero` 跳过扫描。A* 取 Zero 时采用 Dijkstra 式的代价排序，双向 A* 取 Zero 时仍是双向搜索。

双向 A* 首次相遇只获得候选路径，不能立即停止。当前实现等待某一方向有效队首的最小 `f` 不低于候选完整路径代价，再结束搜索；两端工作量合计，同一个格被两端处理会计两次。它可能搜索大量重叠区域，维护双份状态，因而并不保证比普通 A* 或 Dijkstra 更快。相关设计可参见 [Goldberg 与 Harrelson 的论文](https://www.microsoft.com/en-us/research/wp-content/uploads/2005/01/soda05.pdf)，第 5.1 节的对称搜索讨论。

## 4. D* Lite 接口与生命周期

```cpp
#include "dynamic_route/d_star_lite.hpp"

dynamic_route::DStarLite planner{
    map, start, goal, dynamic_route::Connectivity::Four};
auto initial = planner.plan();
planner.set_blocked(obstacle, true);
auto detour = planner.plan();
planner.set_terrain_weight(terrain, 0.5);
auto cheaper = planner.plan();
planner.move_start(new_position);
auto remaining = planner.plan();
```

构造只检查端点，不搜索，也不保证可达。第一次非平凡 `plan()` 才初始化搜索状态。连接方式在构造后固定，目标固定；可选四/八邻接，当前使用 Automatic 距离，没有自定义启发式参数。

规划器借用地图：**地图必须比规划器活得更久**。使用期间不能直接修改、替换或移动地图；动态障碍和权值修改必须经过这个规划器。多个独立规划器需要变化时，应各自使用地图副本和更新流程，避免某个规划器漏掉变化。

| 接口 | 约定 |
| --- | --- |
| `start()` / `goal()` | 读取当前起点和固定目标。 |
| `set_blocked(p,b)` | 立即修改地图并记录变化；阻塞当前起点或目标会抛 `invalid_argument`。 |
| `set_terrain_weight(p,w)` | 立即修改权值并记录变化；先检查边界，再检查有限正数。允许修改端点和障碍格权值，不改变障碍状态。 |
| `move_start(p)` | 移动规划原点，不要求与旧起点相邻；必须在界内且通行。它不是行走过程的模拟。 |
| `plan()` | 应用待处理变化，返回从当前起点到固定目标的最优路径。 |

相同赋值/相同起点不产生新变化；非法参数不改变已有状态。可在第一次规划前更新，也可先批量更新再规划，修复以最终地图为准。起点移动后，原起点不再受端点保护。

不改变地图和起点，再次 `plan()` 可返回路径且不新增搜索工作，但仍要提取路径。当起点位于目标时立即返回，保留尚未处理的地图变化和下界检查，以便以后移开起点时继续处理。

## 5. 从目标向起点传播的含义

D* Lite 保存到目标方向的代价信息，而 Dijkstra/A* 的普通单向实现主要保存从起点出发的代价。理解以下量即可先把握主线：

```text
g[u]   = u 到目标的当前代价估计
rhs[u] = min(c(u,v) + g[v])，v 是 u 的后继
rhs[goal] = 0
key(u) = (min(g[u],rhs[u]) + h(start,u) + km,
          min(g[u],rhs[u]))
```

1. **一致性**：`g==rhs` 时当前估计一致；不同则入队等待处理。
2. **过高估计**：`g>rhs` 时设置 `g=rhs`，向前驱传播改善。
3. **失效估计**：否则先把 `g` 设为无穷，重新更新自己和受影响前驱，撤销不再可靠的旧代价。
4. **结束与路径**：起点一致且剩余队列不足以改善它后结束；从起点依次选择使 `c(u,v)+g[v]` 最小的后继，得到正向路径。

无需每次更新都处理整张地图，但一次局部变化可能传播很远。目标不是让每次修复严格局限于几个格，而是复用仍有效的搜索信息。

## 6. 动态修复、km 和版本号

障碍或权值变化时，setter 把格子加入 `changed_cells_`。Four 更新变化格及四个几何邻格；Eight 更新界内 3×3 区域，覆盖对角线侧边格变化。随后走同一套一致性修复循环，保留 `g_`、`rhs_` 和队列。

这里枚举的是几何邻域：格子刚被阻塞后，合法邻居函数可能返回空，但原来指向它的边仍需失效处理。普通搜索的合法移动仍复用 movement 层。

起点移动时，`key_modifier_`（km）累加旧起点到新起点的启发式距离。已有队列项不全部重排；弹出有效旧项时，若旧 key 小于当前 key，则按新 key 重新入队。节点的 `queue_versions_` 用于丢弃已被更新取代的记录，起点变化本身不使版本过期。重新入队计 pushes，不计节点扩展。

## 7. 权值下界降低的特殊处理

D* Lite 初始化使用所有格子（包括障碍格）的最小权值作为 `minimum_weight_`，这样解封廉价格子不会突然使已有下界失效。它与静态 A* 每次扫描最小通行权值的策略不同。

每个真实权值变化都会记录待修复格；若新权值低于**缓存下界**，另设 `check_lower_bound_`。这里比较的不是单个格子的旧权值。下一次非平凡 `plan()` 会：

1. 扫描最终地图的所有权值。如果批量变化已恢复旧下界，可不重建。
2. 真正出现更低下界时，降低 `minimum_weight_` 并把 km 归零。
3. 清空队列，遍历搜索状态，把全部不一致节点按新 key 入队。
4. 保留 `g/rhs` 和待处理变化，继续普通修复。

降低启发式会让 key 变小，不能只靠处理起点移动时的惰性向上重排。这个特殊分支需要扫描全图/状态数组，不是纯局部操作。实际入队计 pushes，扫描不计扩展，但都消耗运行时间。

权值变大时不提高缓存下界：旧下界仍正确，只是启发式可能更弱。因此 `1→0.25→1→0.25` 只在第一次降低时触发下界重建。

## 8. 浮点数与停止条件

Eight 的 `sqrt(2)` 边费用与 km 多次累加可能使理论上相同的 key 出现微小差异。当前实现保留堆的严格精确排序，在停止判断处保守处理起点第一 key 分量附近的小浮点区间，避免过早停止后保留错误的下游代价。

这可能多处理部分节点。不要直接把堆比较器改成带误差的比较，否则可能破坏严格弱序。混合权值、障碍和起点移动的固定种子测试覆盖过此问题。

## 9. 建议阅读顺序与测试

1. `position.hpp`、`grid_map.hpp/.cpp`、`search_result.hpp`：数据模型。
2. `movement.hpp/.cpp`：移动是否合法、边费用怎么算。
3. `dijkstra.cpp`：队列、距离、旧记录过滤和路径重建。
4. `heuristic.hpp/.cpp`、`a_star.cpp`：下界与搜索方向。
5. `bidirectional_a_star.cpp`：反向费用、候选路径及停止条件。
6. `d_star_lite.hpp/.cpp`：先看 key、update_vertex、plan 主循环，再看更新和重排。
7. `tests/*_test.cpp`：结合具体输入理解边界行为；最后看 benchmarks 的测量流程。

测试覆盖端点合法性、不可达、起点等于目标、地形权值低于 1、四/八邻接、对角线拐角、双向反向费用，以及 D* Lite 障碍/权值/起点混合变化。随机测试使用固定种子，把返回路径合法性和最优代价与 Dijkstra 比较；不是只检查 `found`。

D* Lite 可优先阅读这些测试：`MovingAlongKnownPathReusesGoalDistances`、`MovingOffKnownPathRetainsAndRekeysFrontier`、`LowerBoundChangesAfterStartMovesRetainValidSearch`、`EightConnectivityMixedWeightedUpdatesMatchDijkstra`、`MixedTerrainObstacleAndStartUpdatesMatchDijkstra`。运行方法见 [测试指南](RUNNING.md#4-运行测试)。

理论背景：[Koenig 与 Likhachev 的 D* Lite 技术报告](https://idm-lab.org/bib/abstracts/papers/tr-dstarlite.pdf)。本项目重点是小规模、可解释的独立实现；当前没有 ROS、GUI、CUDA、网络、采样规划或多机器人规划，也未提供地图文件加载、交互式场景编辑器和自动化性能排名工具。
