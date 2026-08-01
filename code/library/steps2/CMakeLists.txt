set(LIBRARY_NAME "steps2")

set(SRC_FILES
        src/camera_info.cpp
        src/feature_extraction.cpp
        src/image_loading.cpp
        src/imu_data_loading.cpp
        src/initialize_calibration.cpp
        src/target_info.cpp
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
        test/feature_extraction.test.cpp
        test/image_loading.test.cpp
        test/imu_data_loading.test.cpp
        test/initialize_calibration.test.cpp
        test/step_runner.test.cpp
        test/target_info.test.cpp
)
AddTests()
