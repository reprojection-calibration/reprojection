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
        tomlplusplus::tomlplusplus
        types
        types_internal
)

# TODO(Jack): This is kind of a hack here to make sure this library only gets installed if we are installing the
# examples. Because for the case where we just want to install the library we do not want the "demos" .so because it is
# specific to the demo applications which consumers of the library do not use. But if we install the library with the
# example applications (ex. for the video-file application release stages) then we need it. It would be nice if this was
# automatically handled by some type of packaging framework and that we did not need to handle it manually here.
set(INSTALL_SO OFF)
if (INSTALL_EXAMPLES)
    set(INSTALL_SO ON)
endif ()
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