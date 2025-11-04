set(LIBRARY_NAME "geometry")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/lie.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        types
)
AddLibrary()

set(TESTS
        test/lie_se3.test.cpp
        test/lie_so3.test.cpp
)
AddTests()