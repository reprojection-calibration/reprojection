set(LIBRARY_NAME "application")

set(SRC_FILES
        src/io.cpp
        src/reprojection_calibration.cpp
        src/utils.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        SQLite::SQLite3
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
        tomlplusplus::tomlplusplus
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
        src/io.test.cpp
        test/reprojection_calibration.test.cpp
)
AddTests()
