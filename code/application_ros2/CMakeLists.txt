cmake_minimum_required(VERSION 3.10)
project(reprojection)

include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

# TODO(Jack): We only need to add rosidl_typesupport_interface and type_description_interfaces because of a failure in
# the dockerized build. The CLion build was working fine! What is going on here?

find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(reprojection REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(type_description_interfaces REQUIRED)

set(INCLUDE_DIRECTORIES
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${rosidl_typesupport_interface_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${type_description_interfaces_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${cv_bridge_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${rosbag2_cpp_LIBRARIES}
        reprojection::application
)

set(LIBRARY_NAME ${PROJECT_NAME})
set(SRC_FILES
        src/reprojection.cpp
)
AddLibrary()

set(EXAMPLES
        examples/reprojection_app.cpp
)
AddExamples()

# WARN(Jack): All other packages follow the library->test->example order, with the LINK_LIBRARIES defined and used by
# the AddLibrary() macro. But for some reason here with ROS2 we enter what smells like an infinite linking loop when we
# follow the normal pattern. Therefore we are forced to put the testing last and set the LINK_LIBRARIES to be empty.
# Technically it seems like just a cosmetic issue, but it might also indicate that we are not fully understanding
# something about the way ROS2 works under the hood (or in our own cmake package logic!).
set(LINK_LIBRARIES "")
find_package(GTest REQUIRED)
enable_testing()
set(TESTS
        test/reprojection.test.cpp
)
AddTests()
