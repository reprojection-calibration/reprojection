set(LIBRARY_NAME "testing_mocks")

set(SRC_FILES
        src/data_generators.cpp
        src/mvg_helpers.cpp
        src/noise_generation.cpp
        src/trajectory.cpp
)
set(PRIVATE_LINK_LIBRARIES
        eigen_utilities
        geometry
        projection_functions
        testing_utilities
        types_internal
)
set(PUBLIC_LINK_LIBRARIES
        spline

)

set(REPROJECTION_INSTALL_TARGET OFF)
AddLibrary()

set(TESTS
        src/mvg_helpers.test.cpp
        src/noise_generation.test.cpp
        src/trajectory.test.cpp
        test/data_generators.test.cpp
)
AddTests()