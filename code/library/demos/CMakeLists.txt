set(LIBRARY_NAME "demos")

set(SRC_FILES
        ../application/src/image_source.cpp
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
        config
        database
        eigen_utilities
        feature_extraction
        geometry
        image_viewer
        optimization
        pnp
        projection_functions
        spline
        steps
        testing_mocks
        testing_utilities
        tomlplusplus::tomlplusplus
        types
        types_internal
)

set(INSTALL_SO OFF)
set(INSTALL_EXAMPLES OFF)

AddLibrary()


set(EXAMPLES
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
        examples/test_data_viz.cpp
)
AddExamples()