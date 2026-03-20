set(LIBRARY_NAME "application")

set(SRC_FILES
        src/reprojection_calibration.cpp
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
        src/step_runner.test.cpp
        src/steps.test.cpp
)
AddTests()
