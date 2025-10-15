set(LIBRARY_NAME "geometry")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/lie.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
)
AddLibrary()

set(TESTS
        test/lie.test.cpp
)
AddTests()