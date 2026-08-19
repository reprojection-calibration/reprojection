set(LIBRARY_NAME "demos")

set(PRIVATE_LINK_LIBRARIES
        ${OpenCV_LIBS}
        ${apriltag_LIBRARIES}
        Ceres::ceres
        Eigen3::Eigen
        SQLite3::SQLite3
        application
        calibration
        config
        database
        eigen_utilities
        feature_extraction
        geometry
        hashing
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
        video_capture
)

set(EXAMPLES
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
)

# TODO(Jack): There is basically no reason we would ever want these examples installed so we hardcode this to OFF. It is
# a problem we have that we do not currently distinguish between development examples like this and the actual
# application executable "example" build in "application/examples". We need to make this delineation more clear.
set(REPROJECTION_INSTALL_EXAMPLES OFF)
AddExamples()