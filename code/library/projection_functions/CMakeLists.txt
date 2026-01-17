set(LIBRARY_NAME "projection_functions")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) # Only required to save projection images

set(SRC_FILES
        src/double_sphere.cpp
        src/pinhole.cpp
        src/pinhole_radtan4.cpp
        src/unified_camera_model.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        eigen_utilities
        ${OpenCV_LIBS}
        types
)
AddLibrary()

set(TESTS
        test/camera_model.test.cpp
        test/double_sphere.test.cpp
        test/image_bounds.test.cpp
        test/pinhole.test.cpp
        test/pinhole_radtan4.test.cpp
        test/unified_camera_model.test.cpp
)
AddTests()
