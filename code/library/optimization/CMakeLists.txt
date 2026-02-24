set(LIBRARY_NAME "optimization")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/angular_velocity_alignment.cpp
        src/camera_imu_nonlinear_refinement.cpp
        src/camera_nonlinear_refinement.cpp
        src/projection_cost_function.cpp
        src/spline_nonlinear_refinement.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/angular_velocity_cost_function.test.cpp
        src/ceres_geometry.test.cpp
        src/projection_cost_function.test.cpp
        src/spline_nonlinear_refinement.test.cpp
        src/spline_projection_cost_function.test.cpp
        test/angular_velocity_alignment.test.cpp
        test/camera_nonlinear_refinement.test.cpp
        test/spline_cost_function.test.cpp
)
AddTests()
