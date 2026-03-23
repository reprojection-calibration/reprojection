set(LIBRARY_NAME "projection_functions")

set(SRC_FILES
        src/double_sphere.cpp
        src/initialize_camera.cpp
        src/pinhole.cpp
        src/pinhole_radtan4.cpp
        src/unified_camera_model.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        ${OpenCV_LIBS}
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        test/camera_model.test.cpp
        test/double_sphere.test.cpp
        test/image_bounds.test.cpp
        test/initialize_camera.test.cpp
        test/pinhole.test.cpp
        test/pinhole_radtan4.test.cpp
        test/unified_camera_model.test.cpp
)
AddTests()
