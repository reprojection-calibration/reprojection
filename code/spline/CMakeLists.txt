set(LIBRARY_NAME "spline")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/se3_spline.cpp
        src/time_handler.cpp
        src/utilities.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        geometry
        types
)
AddLibrary()

set(TESTS
        test/r3_spline.test.cpp
        test/se3_spline.test.cpp
        test/so3_spline.test.cpp
        test/spline_initialization.test.cpp
        test/time_handler.test.cpp
        test/utilities.test.cpp
)
AddTests()