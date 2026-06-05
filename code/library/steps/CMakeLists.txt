set(LIBRARY_NAME "steps")

set(SRC_FILES
        src/bundle_adjustment.cpp
        src/camera_info.cpp
        src/extrinsic_initialization.cpp
        src/extrinsic_optimization.cpp
        src/feature_extraction.cpp
        src/image_loading.cpp
        src/intrinsic_initialization.cpp
        src/pose_initialization.cpp
        src/spline_initialization.cpp
        src/target_info.cpp

)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Ceres::ceres
        Eigen3::Eigen
        SQLite::SQLite3
        caching
        calibration
        config
        database
        feature_extraction
        geometry
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
        test/step_runner.test.cpp
        test/steps.test.cpp
)
AddTests()
