set(LIBRARY_NAME "calibration")

set(SRC_FILES
        src/camera_imu_initialization.cpp
        src/initialization_methods.cpp
        src/intrinsic_initialization.cpp
        src/parabola_line_initialization.cpp
        src/utilities.cpp
        src/linear_pose_initialization.cpp
        src/vanishing_point_initialization.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        logging
        optimization
        pnp
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
        src/intrinsic_initialization.test.cpp
        src/parabola_line_initialization.test.cpp
        src/utilities.test.cpp
        src/vanishing_point_initialization.test.cpp
        test/initialization_methods.test.cpp
)
AddTests()
