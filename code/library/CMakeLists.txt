cmake_minimum_required(VERSION 3.16.3)
project(
        reprojection
        VERSION 0.0.1
        LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic)
endif ()

# Helper macros to prevent insane copy and paste in all subdirectories
include(../cmake_macros/add_examples.cmake)
include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(tomlplusplus REQUIRED)

find_package(GTest REQUIRED)
enable_testing()

# Adopted from https://runebook.dev/en/docs/cmake/prop_tgt/build_rpath_use_origin
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")

add_subdirectory(types_internal)
add_subdirectory(types)
add_subdirectory(testing_utilities)
add_subdirectory(config)
add_subdirectory(caching)
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
add_subdirectory(steps)
add_subdirectory(application)
add_subdirectory(demos)

install(EXPORT reprojectionTargets
        FILE reprojectionTargets.cmake
        NAMESPACE reprojection::
        DESTINATION lib/cmake/reprojection
)
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/reprojectionConfig.cmake
        DESTINATION lib/cmake/reprojection
)