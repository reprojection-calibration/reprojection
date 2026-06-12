set(LIBRARY_NAME "testing_mocks")

set(SRC_FILES
        src/data_generators.cpp
        src/mvg_helpers.cpp
        src/new_sphere_trajectory.cpp
        src/noise_generation.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        testing_utilities
        types_internal
)
set(INSTALL_SO OFF)
AddLibrary()

set(TESTS
        src/mvg_helpers.test.cpp
        src/new_sphere_trajectory.test.cpp
        src/noise_generation.test.cpp
        test/data_generators.test.cpp
)
AddTests()