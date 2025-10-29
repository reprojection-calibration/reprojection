set(LIBRARY_NAME "optimization")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/nonlinear_refinement.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline # needed because of testing_mocks dependency -_-
        testing_mocks
        types
)
AddLibrary()

set(TESTS
        src/ceres_geometry.test.cpp
        src/pinhole_cost_function.test.cpp
        test/nonlinear_refinement.test.cpp
)
AddTests()
