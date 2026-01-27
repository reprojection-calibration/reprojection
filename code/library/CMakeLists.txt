cmake_minimum_required(VERSION 3.28)
project(
        reprojection
        VERSION 0.0.1
        LANGUAGES C CXX
)

option(CODE_COVERAGE "Enable coverage reporting" OFF)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Werror -Wall -Wextra -Wpedantic)

# Helper macros to prevent insane copy and paste in all subdirectories
include(cmake_macros/add_library.cmake)
include(cmake_macros/add_tests.cmake)

find_package(GTest REQUIRED)
enable_testing()

add_subdirectory(types)
add_subdirectory(testing_utilities)
add_subdirectory(config)
add_subdirectory(calibration_data_views)
add_subdirectory(database)
add_subdirectory(eigen_utilities)
add_subdirectory(geometry)
add_subdirectory(projection_functions)
add_subdirectory(spline)
add_subdirectory(testing_mocks)
add_subdirectory(optimization)
add_subdirectory(pnp)
add_subdirectory(calibration)
add_subdirectory(feature_extraction)
add_subdirectory(demos)
