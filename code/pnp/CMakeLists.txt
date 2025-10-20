set(LIBRARY_NAME "pnp")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/dlt.cpp
        src/dlt_matrix_decompositions.cpp
        src/matrix_utilities.cpp
        src/multiple_view_geometry_data_generator.cpp
        src/nonlinear_refinement.cpp
        src/plane_utilities.cpp
        src/pnp.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        geometry
        testing_mocks
)
AddLibrary()

set(TESTS
        src/dlt.test.cpp
        src/dlt_matrix_decompositions.test.cpp
        src/matrix_utilities.test.cpp
        src/multiple_view_geometry_data_generator.test.cpp
        src/nonlinear_refinement.test.cpp
        src/plane_utilities.test.cpp
        src/temp_geometric_transforms.test.cpp
        test/pnp.test.cpp
)
AddTests()