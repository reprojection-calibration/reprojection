cmake_minimum_required(VERSION 3.10)
project(reprojection)

# NOTE(Jack): We cannot enforce c++ 20 here because noetic and its boosts dependency are quite old and apparently not
# compatible with our build environments gcc13.
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wno-deprecated-enum-enum-conversion)
endif ()

include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

find_package(cv_bridge REQUIRED)
find_package(reprojection REQUIRED)
find_package(rosbag REQUIRED)
find_package(sensor_msgs REQUIRED)

set(INCLUDE_DIRECTORIES
        ${OpenCV_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OpenCV_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${rosbag_LIBRARIES}
        ${std_msgs_LIBRARIES}
        reprojection::application
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
AddExamples()

