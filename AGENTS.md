# DynamicRoute Project Guide

## Project intent

DynamicRoute is a new, compact C++17 path-planning project intended for interview discussion and individual maintenance. Favor code whose design, invariants, tradeoffs, and algorithms can be clearly explained by one developer.

The nearby `path_planning` repository is reference material only. It is not part of DynamicRoute.

## Repository boundaries

- Never modify `path_planning` or any other reference repository.
- Do not copy the reference project's architecture or implementation wholesale.
- Consult reference code only to understand an algorithm or compare a specific design choice.
- Implement DynamicRoute independently, with its own naming, interfaces, tests, and documentation.

## Scope

Core data model:

- `Position`
- `GridMap`
- `SearchResult`
- `SearchStats`

Planned algorithms:

- Dijkstra
- A*
- Bidirectional A*
- D* Lite

Planned capabilities:

- Weighted terrain
- 4-connected and 8-connected movement
- Configurable heuristics
- Dynamic obstacles
- Benchmarking
- GoogleTest
- CMake

Explicit non-goals:

- ROS
- Qt or another GUI
- CUDA
- Networking
- Multi-agent planning
- Genetic algorithms
- Ant colony optimization
- RRT or RRT*

## Engineering principles

- Use C++17.
- Prefer a small number of concrete, cohesive types over framework-style abstractions.
- Add an abstraction only when a current use case requires it.
- Keep public APIs narrow and make invalid states difficult to represent.
- Use value semantics and standard-library types by default.
- Keep algorithm-specific state out of `GridMap` and other core domain types.
- Avoid global mutable state, hidden ownership, and unnecessary inheritance.
- Treat warnings as useful feedback and keep builds warning-clean where practical.
- Document non-obvious invariants and design decisions, not line-by-line mechanics.

## Incremental workflow

- Work in small, reviewable steps; do not generate the whole project at once.
- Do not implement an algorithm before its supporting data model and tests are agreed upon.
- After every meaningful code or build-system change, configure/build the project and run the relevant tests.
- Add or update focused tests with each behavior change.
- Keep benchmarks separate from correctness tests.
- Do not add features outside the stated scope without explicit agreement.
- Complete the final documentation stage in one pass when reached, as requested
  by the user. Include reproducible setup, build/run commands, full and filtered
  test runs, result locations, output interpretation, and VS Code usage.

## Current milestone boundary

Static planners, shared heuristics, and the D* Lite dynamic-update milestone are
complete. DStarLite supports Four/Eight connectivity, obstacle/terrain updates,
and start movement, validated against Dijkstra. Preserve the invariants below.

The current user-authorized increment adds static path visualization for
Dijkstra, A*, and Bidirectional A*. Keep it as an optional CMake example under
examples, linked to the core library. Export a self-contained HTML/SVG report
showing the same fixed map, obstacles, endpoints, final routes, and search stats.
Validate paths against Dijkstra before export. No GUI framework, algorithm
changes, benchmark timing changes, or dynamic animation are part of this step.

The final documentation milestone is complete. Keep the
Chinese README as the entry point, docs/RUNNING.md for reproducible operations,
docs/ALGORITHMS.md for APIs and reading notes, and docs/BENCHMARKS.md for fixtures,
measurement boundaries, and interpretation. Preserve the six completed dynamic
scenarios and algorithm behavior. Keep unrelated features and optimizations
outside this increment. Document the new visualization build/run commands.

- Verify documented Debug/Release configuration and builds, full and filtered
  tests, both benchmarks, and result saving/filtering on the current toolchain.
- Keep publishable documentation independent of workstation layout: use paths
  relative to the repository root and discover tools through PATH. Never commit
  personal directory names, machine-specific compiler paths, credentials, build
  outputs, or local editor settings. Include C++17, the first-configure
  GoogleTest download requirement, and generic VS Code troubleshooting.
- docs/examples/basic_usage.cpp is a documentation example, built independently
  against the existing core library as described in RUNNING.md. It prints paths,
  costs, and stats for static planning and DStarLite updates; it is not a CTest
  test or an additional default CMake target.
- Distinguish cached-dependency verification from a fresh network download, and
  command-line verification from unverified VS Code UI operations. Date the
  verification record and keep commands consistent with current target names.

- Build benchmarks only with DYNAMIC_ROUTE_BUILD_BENCHMARKS=ON (default OFF).
  Keep the executable under benchmarks and outside CTest and algorithm code.
- Use Release for timing, fixed inputs/seeds/repetitions, and steady_clock.
  Generate the map once outside timing; guarantee a route for this baseline.
- Construct a fresh DStarLite per measured call. Include constructor, first
  planning, and planner cleanup; do not compare cached repeat calls to fresh
  static searches in the static benchmark.
- In the dynamic benchmark, retain one DStarLite per trial. Make independent
  map copies for each algorithm/trial outside timing. Precompute per-event
  Dijkstra reference costs outside timing. Include map setters or move_start
  in update time. Build shared start sequences outside timing.
- Report initial planning, average update time, and initial-plus-all-updates time
  separately, plus first_update_us and mean_later_update_us. Dynamic total_us
  excludes map copies, validation, output, result
  destruction, and final persistent-planner teardown; document that boundary.
- Use the same update sequence for every algorithm within a scenario. Share
  apply_update between reference maps, static-planner maps, and DStarLite; use
  the planner setter for persistent state. Start movement uses a shared starts
  sequence, with fixed starts for the existing scenarios. Keep further
  scenarios and algorithm optimizations outside this increment.
  Preserve unfavorable baselines as well as favorable cases; do not change
  inputs just to obtain a speedup.
- Reuse run_scenario, run_trial, and measure across all six scenarios. Include
  scenario in each result row and report map dimensions, endpoints' fixture,
  changed cell, update kind, terrain values where relevant, and pre/post-update
  reference costs in the documentation/metadata.
  For start movement report initial/final starts, goal, sequence source, and
  initial/first-update/final reference costs instead of a changed cell.
  Verify that an explicit off-path update is not on the original Dijkstra path.
  The two open cases isolate update location; comparing them with the weighted
  case changes map/weights/endpoints too and is not a controlled location test.
- Validate reachability, path legality, reconstructed cost, and Dijkstra cost
  outside timing. Report failure with a nonzero exit code.
- Report scenario metadata, total cost, expanded nodes, frontier pushes, and
  average microseconds. Describe the measurement boundary and limited scope;
  do not draw universal algorithm rankings from these limited scenarios.

- Support Four and Eight connectivity with Four as the constructor default.
  Support finite positive terrain weights, including weights below 1.0. Defer
  configurable D* Lite heuristics.
- Reuse movement-layer neighbors, diagonal costs, and corner checks. Use shared
  Automatic heuristic distances: Manhattan for Four and Octile for Eight.
  Apply the selected connectivity to both queue keys and start-movement km.
- DStarLite borrows a GridMap that must outlive it. Callers must not modify,
  replace, or move that map directly while the planner is in use. Obstacle/terrain
  updates go through the planner so search state cannot silently become stale. A changing
  map must not be shared between planners with independent search state.
- Keep the goal fixed. move_start accepts any unblocked in-bounds position;
  it changes the planning origin and does not simulate travel to that position.
  Reject invalid positions before changing any state; identical moves do nothing.
- Keep persistent algorithm state in DStarLite, never in GridMap. The existing
  static planners keep their per-call state and remain independently readable.
- Match existing endpoint validation: check both bounds before obstacle state;
  reject blocked endpoints even when start == goal. Valid disconnected endpoints
  may be constructed; plan() reports failure through SearchResult.
- Preserve fixed-map planning tests for costs, valid reconstructed paths,
  asymmetric weights, weights below one, and repeated calls without search work.
- set_blocked changes the map immediately and defers repair to plan(). Reject
  out-of-bounds updates and blocking either endpoint before changing the map.
  Repeated assignments of the existing obstacle state do nothing. Multiple
  updates before plan() are supported; process the final map state.
- set_terrain_weight follows GridMap validation: bounds before finite positive
  weight, with no state changes for rejected or identical updates. Permit changes
  on endpoints and blocked cells without altering their obstacle state. Update
  the map immediately and defer search repair to plan(), sharing changed_cells.
- Retain g/rhs and frontier across obstacle changes. Four updates the changed
  cell and four geometric neighbors; Eight updates its bounded 3x3 neighborhood,
  including blocked cells. This also repairs diagonals whose side cells changed.
  Do not restart search.
- Initially use the minimum terrain weight over ALL cells, including blocked
  cells, as a heuristic lower bound. Keep this bound when weights increase.
  If a recorded weight falls below it, check the final map's minimum at plan().
  On a genuine decrease, lower the bound, reset km, and rebuild the frontier from
  inconsistent cells using current keys; retain g/rhs and pending map changes.
  This exceptional rekey scans all cells and is not purely local repair. Count
  actual queue pushes in stats; scans do not count as node expansions.
- move_start accumulates the heuristic distance from old to new start in the
  key modifier (km). Start movement alone retains the frontier. Reinsert
  valid entries whose stored keys are smaller than their current keys.
- Use per-cell queue versions to discard superseded records. A changed start
  alone must not invalidate a queue record. Rekey pushes count in SearchStats;
  rekeying without vertex processing does not count as a node expansion.
- At the goal, plan() returns immediately with zero work and retains pending
  obstacle/terrain changes and lower-bound checks if the start later moves away.
- Keep heap ordering exact. At termination, conservatively finish a small
  floating-point band around the start's first key component to avoid stopping
  before invalidating stale downstream costs after diagonal moves and km sums.
  Preserve the seeded mixed-update regression exercising this rounding case.
- Keep further benchmark expansion and unrelated refactoring outside this increment.
- Reuse destination-weight edge costs; backward propagation must preserve the
  original forward edge's cost. Keep heuristic bounds valid for weights below 1.
- Return SearchResult from plan(). SearchStats describe
  work performed by that call, rather than lifetime totals.
- Keep configurable D* Lite heuristics and unrelated refactoring
  outside this milestone.
