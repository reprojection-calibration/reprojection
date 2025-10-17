set(LIBRARY_NAME "pnp")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/camera_matrix_decomposition.cpp
        src/dlt.cpp
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
)
AddLibrary()

set(TESTS
        src/camera_matrix_decomposition.test.cpp
        src/dlt.test.cpp
        src/matrix_utilities.test.cpp
        src/multiple_view_geometry_data_generator.test.cpp
        src/nonlinear_refinement.test.cpp
        src/plane_utilities.test.cpp
        src/temp_geometric_transforms.test.cpp
        test/pnp.test.cpp
)
AddTests()