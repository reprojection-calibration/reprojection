set(LIBRARY_NAME "spline")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/r3_spline.cpp
        src/se3_spline.cpp
        src/so3_spline.cpp
        src/time_handler.cpp
        src/utilities.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        geometry
)
AddLibrary()

set(TESTS
        src/utilities.test.cpp
        test/r3_spline.test.cpp
        test/se3_spline.test.cpp
        test/so3_spline.test.cpp
        test/time_handler.test.cpp
)
AddTests()