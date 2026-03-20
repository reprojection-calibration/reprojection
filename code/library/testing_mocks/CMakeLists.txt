set(LIBRARY_NAME "testing_mocks")

set(SRC_FILES
        src/data_generator_helpers.cpp
        src/imu_data_generator.cpp
        src/mvg_data_generator.cpp
        src/mvg_helpers.cpp
        src/noise_generation.cpp
        src/sphere_trajectory.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        testing_utilities
        types
)
set(INSTALL_SO OFF)
AddLibrary()

set(TESTS
        src/data_generator_helpers.test.cpp
        src/mvg_helpers.test.cpp
        src/noise_generation.test.cpp
        src/sphere_trajectory.test.cpp
        test/imu_data_generator.test.cpp
        test/mvg_data_generator.test.cpp
)
AddTests()