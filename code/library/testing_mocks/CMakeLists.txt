set(LIBRARY_NAME "testing_mocks")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/imu_data_generator.cpp
        src/mvg_data_generator.cpp
        src/mvg_helpers.cpp
        src/noise_generation.cpp
        src/sphere_trajectory.cpp
)
set(LINK_LIBRARIES
        database # REMOVE!!!
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/mvg_helpers.test.cpp
        src/noise_generation.test.cpp
        src/sphere_trajectory.test.cpp
        test/imu_data_generator.test.cpp
        test/imu_spline_data_analysis.test.cpp
        test/mvg_data_generator.test.cpp
)
AddTests()