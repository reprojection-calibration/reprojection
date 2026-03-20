set(LIBRARY_NAME "application")

set(SRC_FILES
        src/steps.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        caching
        calibration
        database
        optimization
        projection_functions # Introduced due to adding EstimatePoseViaPinholePnP to the linear_pose_init public header interface.
        spline # Required because of the EstimateCameraImuRotationAndGravity() interface!
        types
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_mocks
        testing_utilities
)
set(TESTS
        test/step_runner.test.cpp
        test/steps.test.cpp
)
AddTests()
