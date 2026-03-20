set(LIBRARY_NAME "demos")

set(SRC_FILES
        src/image_source.cpp
)
set(LINK_LIBRARIES
        application
        calibration
        Ceres::ceres
        database
        Eigen3::Eigen
        eigen_utilities
        feature_extraction
        geometry
        types
        ${apriltag_LIBRARIES}
        optimization
        ${OpenCV_LIBS}
        pnp
        projection_functions
        spline
)
AddLibrary()

set(TESTS
        test/image_source.test.cpp
)
AddTests()

set(EXAMPLES
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
)
AddExamples()