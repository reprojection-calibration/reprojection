set(LIBRARY_NAME "application")

set(SRC_FILES
        src/load_and_validate_config.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        caching
        calibration
        config
        database
        optimization
        projection_functions # Introduced due to adding EstimatePoseViaPinholePnP to the linear_pose_init public header interface.
        spline # Required because of the EstimateCameraImuRotationAndGravity() interface!
        steps
        types
        types_internal
)
set(INSTALL_HEADERS ON)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_mocks
        testing_utilities
)
set(TESTS
        test/cli_utils.test.cpp
        test/load_and_validate_config.test.cpp
)
AddTests()
