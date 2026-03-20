cmake_minimum_required(VERSION 3.10)
project(reprojection)

include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(reprojection REQUIRED)

set(INCLUDE_DIRECTORIES
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
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
