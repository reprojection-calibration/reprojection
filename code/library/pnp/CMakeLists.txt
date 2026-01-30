set(LIBRARY_NAME "pnp")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/dlt.cpp
        src/dlt_matrix_decompositions.cpp
        src/matrix_utilities.cpp
        src/plane_utilities.cpp
        src/pnp.cpp
)
set(LINK_LIBRARIES
        calibration_data_views
        Eigen3::Eigen
        eigen_utilities
        geometry
        optimization
        projection_functions
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/dlt.test.cpp
        src/dlt_matrix_decompositions.test.cpp
        src/matrix_utilities.test.cpp
        src/plane_utilities.test.cpp
        test/pnp.test.cpp
)
AddTests()