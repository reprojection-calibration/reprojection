cmake_minimum_required(VERSION 3.10)
project(reprojection)

include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

find_package(reprojection REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DIRECTORIES
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        reprojection::application
)

set(LIBRARY_NAME ${PROJECT_NAME})
set(SRC_FILES
        src/reprojection.cpp
)
AddLibrary()

find_package(GTest REQUIRED)
enable_testing()
set(TESTS
        test/reprojection.test.cpp
)
AddTests()

set(EXAMPLES
        examples/reprojection_app.cpp
)
AddExamples()

