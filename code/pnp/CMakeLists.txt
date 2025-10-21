set(LIBRARY_NAME "pnp")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/dlt.cpp
        src/dlt_matrix_decompositions.cpp
        src/matrix_utilities.cpp
        src/nonlinear_refinement.cpp
        src/plane_utilities.cpp
        src/pnp.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline # needed because of testing_mocks dependency -_-
        testing_mocks
)
AddLibrary()

set(TESTS
        src/dlt.test.cpp
        src/dlt_matrix_decompositions.test.cpp
        src/matrix_utilities.test.cpp
        src/nonlinear_refinement.test.cpp
        src/plane_utilities.test.cpp
        test/pnp.test.cpp
)
AddTests()