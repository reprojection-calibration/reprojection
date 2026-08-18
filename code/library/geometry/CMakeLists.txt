set(LIBRARY_NAME "geometry")

set(SRC_FILES
        src/lie.cpp
)
set(PRIVATE_LINK_LIBRARIES
        Eigen3::Eigen
        types_internal
)
set(PUBLIC_LINK_LIBRARIES
        Ceres::ceres
)
AddLibrary()

set(TESTS
        test/lie_se3.test.cpp
        test/lie_so3.test.cpp
)
AddTests()