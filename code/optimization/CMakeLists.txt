set(LIBRARY_NAME "optimization")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/nonlinear_refinement.cpp
        src/projection_cost_function.cpp
        src/r3_spline_cost_function.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        geometry
        projection_functions
        spline
        testing_mocks
        types
)
AddLibrary()

set(TESTS
        src/ceres_geometry.test.cpp
        src/projection_cost_function.test.cpp
        src/r3_spline_cost_function.test.cpp
        #src/so3_spline_cost_function.test.cpp
        test/camera_nonlinear_refinement.test.cpp
        test/r3_spline_nonlinear_refinement.test.cpp
)
AddTests()
