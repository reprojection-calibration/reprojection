set(LIBRARY_NAME "projection_functions")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) # Only required to save projection images

set(SRC_FILES
        src/pinhole.cpp
        src/pinhole_radtan4.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        ${OpenCV_LIBS}
        types
)
AddLibrary()

set(TESTS
        test/double_sphere.test.cpp
        test/pinhole.test.cpp
        test/pinhole_radtan4.test.cpp
)
AddTests()
