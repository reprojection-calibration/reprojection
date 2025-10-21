set(LIBRARY_NAME "projection_functions")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/pinhole.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
)
AddLibrary()

set(TESTS
        test/double_sphere.test.cpp
        test/pinhole.test.cpp
)
AddTests()
