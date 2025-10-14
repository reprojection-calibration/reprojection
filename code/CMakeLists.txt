cmake_minimum_required(VERSION 3.28)

project(
        reprojection
        VERSION 0.0.1
        LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Werror -Wall -Wextra -Wpedantic)

# Helper macros to prevent insane copy and paste in all subdirectories
include(cmake_macros/add_library.cmake)
include(cmake_macros/add_tests.cmake)

message(STATUS "Starting to FetchContent googletest - this requires a network connection. ")
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

add_subdirectory(geometry)
