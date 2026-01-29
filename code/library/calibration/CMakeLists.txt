set(LIBRARY_NAME "calibration")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) # REMOVE when/if database is removed

set(SRC_FILES
        src/parabola_line_initialization.cpp
        src/linear_pose_initialization.cpp
        src/vanishing_point_initialization.cpp
)
set(LINK_LIBRARIES
        calibration_data_views
        Eigen3::Eigen
        eigen_utilities
        geometry
        pnp
        projection_functions
        spline
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/parabola_line_initialization.test.cpp
        src/vanishing_point_initialization.test.cpp
        test/linear_pose_initialization.test.cpp
        test/spline_initialization.test.cpp
)
AddTests()
