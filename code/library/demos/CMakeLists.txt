set(LIBRARY_NAME "demos")

set(SRC_FILES
        src/image_source.cpp
)
set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        ${apriltag_LIBRARIES}
        Ceres::ceres
        Eigen3::Eigen
        SQLite::SQLite3
        application
        caching
        calibration
        database
        eigen_utilities
        feature_extraction
        geometry
        optimization
        pnp
        projection_functions
        spline
        steps
        tomlplusplus::tomlplusplus
        types
        types_internal
)
set(INSTALL_SO OFF)
AddLibrary()

set(TESTS
        test/image_source.test.cpp
)
AddTests()

set(EXAMPLES
        examples/calibration.cpp
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
)
AddExamples()