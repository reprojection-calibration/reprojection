set(LIBRARY_NAME "steps2")

set(SRC_FILES
        src/image_loading.cpp
        src/initialize_calibration.cpp
)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Ceres::ceres
        Eigen3::Eigen
        SQLite::SQLite3
        calibration
        config
        database
        feature_extraction
        geometry
        hashing
        image_viewer
        logging
        optimization
        spline
        types
        tomlplusplus::tomlplusplus
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_mocks
        testing_utilities
)
set(TESTS
        test/image_loading.test.cpp
        test/initialize_calibration.test.cpp
        test/step_runner.test.cpp
)
AddTests()
