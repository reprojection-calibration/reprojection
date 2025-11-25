set(LIBRARY_NAME "types")

find_package(Eigen3 REQUIRED)

# Header only package, therefore "INTERFACE" and we do not use the default cmake_macros
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
)

set(LINK_LIBRARIES
        Eigen3::Eigen
)

set(TESTS
        test/calibration_types.test.cpp
)
AddTests()