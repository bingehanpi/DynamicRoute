cmake_minimum_required(VERSION 3.20)

project(
    DynamicRoute
    VERSION 0.1.0
    DESCRIPTION "A compact C++ path-planning project"
    LANGUAGES CXX
)

add_library(
    dynamic_route_core
    src/a_star.cpp
    src/bidirectional_a_star.cpp
    src/d_star_lite.cpp
    src/dijkstra.cpp
    src/grid_map.cpp
    src/heuristic.cpp
    src/movement.cpp
)
add_library(DynamicRoute::core ALIAS dynamic_route_core)

target_compile_features(dynamic_route_core PUBLIC cxx_std_17)
target_include_directories(
    dynamic_route_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

option(DYNAMIC_ROUTE_BUILD_BENCHMARKS "Build standalone planning benchmarks" OFF)

if(DYNAMIC_ROUTE_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

option(DYNAMIC_ROUTE_BUILD_EXAMPLES "Build path visualization examples" OFF)

if(DYNAMIC_ROUTE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
