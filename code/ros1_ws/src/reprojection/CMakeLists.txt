cmake_minimum_required(VERSION 3.10)
project(reprojection)

add_compile_options(-std=c++20)

# Find ROS dependencies manually
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories for headers
include_directories(
        include
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)

# Build your library
add_library(reprojection_lib src/reprojection.cpp)

# Build your executable
add_executable(reprojection_app src/reprojection_app.cpp)
target_link_libraries(reprojection_app
        reprojection_lib
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
)

# Build test using plain CMake + GTest
find_package(GTest REQUIRED)
add_executable(reprojection_test test/reprojection.test.cpp)
target_link_libraries(reprojection_test reprojection_lib GTest::Main
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
)
enable_testing()
include(GoogleTest)
gtest_discover_tests(reprojection_test)

