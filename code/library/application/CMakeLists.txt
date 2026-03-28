set(LIBRARY_NAME "application")

set(SRC_FILES
        src/calibrate.cpp
        src/io.cpp
        src/utils.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        caching
        calibration
        config
        database
        logging
        optimization
        projection_functions # Introduced due to adding EstimatePoseViaPinholePnP to the linear_pose_init public header interface.
        spdlog::spdlog
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
        src/utils.test.cpp
        test/calibrate.test.cpp
        test/io.test.cpp
)
AddTests()
