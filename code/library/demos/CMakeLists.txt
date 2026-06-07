set(LIBRARY_NAME "demos")

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        ${apriltag_LIBRARIES}
        Ceres::ceres
        Eigen3::Eigen
        SQLite::SQLite3
        application
        hashing
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

# NOTE(Jack): We have an unclear separation of what is an internal example application and what a external user facing
# application is. Right now both of those are just considered "examples". Here in the demos folder we have internal only
# examples - i.e. we would never install them inside a docker and release stage and give them to a user. The public
# facing calibration "example" is found in the application folder. Due to this confusing, unclear, and arbitrary split
# of different types of "examples" we need ot hardcode INSTALL_EXAMPLES to OFF here. The bottom line is that we do not
# want to install anything the app user does not use! Will this be easy to maintain or easy for future developers to
# understand? No.
set(INSTALL_EXAMPLES OFF)

set(EXAMPLES
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
        examples/test_data_viz.cpp
)
AddExamples()