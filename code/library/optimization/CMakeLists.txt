set(LIBRARY_NAME "optimization")

set(SRC_FILES
        src/angular_velocity_alignment.cpp
        src/bundle_adjustment.cpp
        src/camera_imu_calibration.cpp
        src/cost_functions/reprojection_error.cpp
        src/cost_functions/reprojection_error_spline.cpp
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
        src/cost_functions/reprojection_error.test.cpp
        src/cost_functions/reprojection_error_spline.test.cpp
        src/cost_functions/rigid_body_angular_velocity.test.cpp
        test/angular_velocity_alignment.test.cpp
        test/bundle_adjustment.test.cpp
        test/camera_imu_calibration.test.cpp
)
AddTests()
