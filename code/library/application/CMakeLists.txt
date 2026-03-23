set(LIBRARY_NAME "application")

set(SRC_FILES
        src/load_and_validate_config.cpp
        src/steps.cpp
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
        test/load_and_validate_config.test.cpp
        test/step_runner.test.cpp
        test/steps.test.cpp
)
AddTests()
