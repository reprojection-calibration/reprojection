set(LIBRARY_NAME "calibration")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) # REMOVE when/if database is removed

set(SRC_FILES
        src/parabola_line_initialization.cpp
        src/vanishing_point_initialization.cpp
)
set(INCLUDE_DIRECTORIES
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        database
        Eigen3::Eigen
        eigen_utilities
        geometry
        ${OpenCV_LIBS}
        optimization
        pnp
        projection_functions
        spline
        types
)
AddLibrary()

set(TESTS
        src/linear_pose_initialization.test.cpp
        src/parabola_line_initialization.test.cpp
        src/vanishing_point_initialization.test.cpp
)
AddTests()
