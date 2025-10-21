set(LIBRARY_NAME "testing_mocks")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/mvg_generator.cpp
        src/sphere_trajectory.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        geometry
        spline
)
AddLibrary()

set(TESTS
        src/sphere_trajectory.test.cpp
        test/mvg_generator.test.cpp
)
AddTests()