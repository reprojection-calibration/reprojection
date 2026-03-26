set(LIBRARY_NAME "steps")

set(SRC_FILES
        src/camera_nonlinear_refinement.cpp
        src/feature_extraction.cpp
        src/intrinisic_initialization.cpp # TODO FIX NAME SPELLING!!!
        src/linear_pose_initialization.cpp

)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Eigen3::Eigen
        caching
        calibration
        database
        feature_extraction
        optimization
        spline # transitive dependency from "calibration/initialization_methods.hpp" - should be removed!
        types
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
