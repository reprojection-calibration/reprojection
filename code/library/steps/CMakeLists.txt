set(LIBRARY_NAME "steps")

set(SRC_FILES
        src/camera_info.cpp
        src/camera_nonlinear_refinement.cpp
        src/feature_extraction.cpp
        src/image_loading.cpp
        src/intrinsic_initialization.cpp
        src/linear_pose_initialization.cpp

)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Ceres::ceres
        Eigen3::Eigen
        SQLite::SQLite3
        caching
        calibration
        database
        feature_extraction
        optimization
        spline # transitive dependency from "calibration/initialization_methods.hpp" - should be removed!
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
