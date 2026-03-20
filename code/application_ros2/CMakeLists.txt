cmake_minimum_required(VERSION 3.10)
project(reprojection)

add_compile_options(-std=c++20 -Wall -Wextra -Wpedantic)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(reprojection_lib src/reprojection.cpp)
target_include_directories(reprojection_lib PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(reprojection_lib
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
)

add_executable(reprojection_app src/reprojection_app.cpp)
target_include_directories(reprojection_app PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(reprojection_app
        reprojection_lib
)

find_package(GTest REQUIRED)
enable_testing()
include(GoogleTest)
add_executable(reprojection_test test/reprojection.test.cpp)
target_include_directories(reprojection_test PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(reprojection_test
        reprojection_lib
        GTest::Main
)
gtest_discover_tests(reprojection_test)