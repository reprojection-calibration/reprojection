cmake_minimum_required(VERSION 3.28)

set(LIBRARY_NAME "spline")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/r3_spline.cpp
        src/se3_spline.cpp
        src/so3_spline.cpp
        src/utilities.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        geometry
)
AddLibrary()

set(TESTS
        src/r3_spline.test.cpp
        src/se3_spline.test.cpp
        src/so3_spline.test.cpp
        src/utilities.test.cpp
)
AddTests()