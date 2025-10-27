set(LIBRARY_NAME "calibration")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/vanishing_point_initialization.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        projection_functions
        types
)
AddLibrary()

set(TESTS
        src/vanishing_point_initialization.test.cpp
)
AddTests()
