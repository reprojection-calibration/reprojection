set(LIBRARY_NAME "calibration_data_views")

# TODO(Jack): If the source code here keeps getting more abstract and complicated consider making this a normal
# non-header only package with source code file splits and testing.

find_package(Eigen3 REQUIRED)

# Header only package, therefore "INTERFACE" and we do not use the default cmake macros that we use for all the others.
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
)
target_link_libraries(${LIBRARY_NAME} INTERFACE
        Eigen3::Eigen
        types
)


set(TESTS
        test/optimization_view.test.cpp
)
AddTests()