set(LIBRARY_NAME "application")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/steps.cpp
)

set(LINK_LIBRARIES
        caching
        calibration
        database
        Eigen3::Eigen
        optimization
        projection_functions # Introduced due to adding EstimatePoseViaPinholePnP to the linear_pose_init public header interface.
        spline # Required because of the EstimateCameraImuRotationAndGravity() interface!
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        test/step_runner.test.cpp
        test/steps.test.cpp
)
AddTests()
