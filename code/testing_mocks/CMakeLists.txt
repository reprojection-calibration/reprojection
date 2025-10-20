set(LIBRARY_NAME "testing_mocks")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/mvg_generator.cpp
        src/sphere_trajectory.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        geometry
        spline
)
AddLibrary()

set(TESTS
        src/sphere_trajectory.test.cpp
)
AddTests()