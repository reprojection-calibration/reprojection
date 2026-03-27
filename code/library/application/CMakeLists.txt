set(LIBRARY_NAME "application")

set(SRC_FILES
        src/calibrate.cpp
        src/io.cpp
)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Eigen3::Eigen
        caching
        calibration
        config
        database
        feature_extraction
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
        test/io.test.cpp
)
AddTests()
