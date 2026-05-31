set(LIBRARY_NAME "optimization")

set(SRC_FILES
        src/angular_velocity_alignment.cpp
        src/camera_imu_nonlinear_refinement.cpp
        src/camera_nonlinear_refinement.cpp
        src/cost_functions/projection_cost_function.cpp
        src/cost_functions/spline_projection_cost_function.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_mocks
        testing_utilities
)
set(TESTS
        src/ceres_geometry.test.cpp
        src/cost_functions/angular_velocity_cost_function.test.cpp
        src/cost_functions/projection_cost_function.test.cpp
        src/cost_functions/spline_projection_cost_function.test.cpp
        test/angular_velocity_alignment.test.cpp
        test/camera_imu_nonlinear_refinement.test.cpp
        test/camera_nonlinear_refinement.test.cpp
)
AddTests()
