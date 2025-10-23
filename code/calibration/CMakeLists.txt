set(LIBRARY_NAME "calibration")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/focal_length_initialization.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        projection_functions
        types
)
AddLibrary()

set(TESTS
        src/focal_length_initialization.test.cpp
)
AddTests()
