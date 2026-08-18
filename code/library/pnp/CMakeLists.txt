set(LIBRARY_NAME "pnp")

set(SRC_FILES
        src/dlt.cpp
        src/dlt_matrix_decompositions.cpp
        src/matrix_utilities.cpp
        src/plane_utilities.cpp
        src/pnp.cpp
)
set(LINK_LIBRARIES
        # NOTE(Jack): We use raw eigen functions so much in this library that it seems wrong to depend on the transitive
        # eigen inclusion from types_internal or any other library.
        Eigen3::Eigen
        eigen_utilities
        optimization
        projection_functions
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        spline # REQUIRED BECAUSE OF TESTING_MOCKS INTERFACE!
        testing_mocks
        testing_utilities
)
set(TESTS
        src/dlt.test.cpp
        src/dlt_matrix_decompositions.test.cpp
        src/matrix_utilities.test.cpp
        src/plane_utilities.test.cpp
        test/pnp.test.cpp
)
AddTests()