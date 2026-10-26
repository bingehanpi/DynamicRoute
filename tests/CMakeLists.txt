include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.17.0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(
    dynamic_route_tests
    a_star_test.cpp
    bidirectional_a_star_test.cpp
    d_star_lite_test.cpp
    dijkstra_test.cpp
    grid_map_test.cpp
    heuristic_test.cpp
    movement_test.cpp
    position_test.cpp
    search_result_test.cpp
    smoke_test.cpp
)
target_link_libraries(
    dynamic_route_tests
    PRIVATE
        DynamicRoute::core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(dynamic_route_tests)
