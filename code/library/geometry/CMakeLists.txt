set(LIBRARY_NAME "geometry")

set(SRC_FILES
        src/lie.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        types_internal
)
AddLibrary()

set(TESTS
        test/lie_se3.test.cpp
        test/lie_so3.test.cpp
)
AddTests()