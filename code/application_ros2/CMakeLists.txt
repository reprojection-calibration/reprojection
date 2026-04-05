cmake_minimum_required(VERSION 3.10)
project(application_ros2)

include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

# Adopted from https://runebook.dev/en/docs/cmake/prop_tgt/build_rpath_use_origin
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")

find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(reprojection REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tomlplusplus REQUIRED)

set(INCLUDE_DIRECTORIES
        ${sensor_msgs_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        cv_bridge::cv_bridge
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp
        reprojection::application
        tomlplusplus::tomlplusplus
)

set(LIBRARY_NAME ${PROJECT_NAME})
set(SRC_FILES
        src/image_loading.cpp
        src/reprojection.cpp
)
AddLibrary()

find_package(GTest REQUIRED)
enable_testing()
set(TESTS
        src/image_loading.test.cpp
        test/reprojection.test.cpp
)
AddTests()

set(EXAMPLES
        examples/reprojection_app.cpp
)
set(INSTALL_EXAMPLES ON)
AddExamples()
