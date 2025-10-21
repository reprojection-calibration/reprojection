set(LIBRARY_NAME "projection_functions")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/pinhole.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
)
AddLibrary()

set(TESTS
        test/pinhole.test.cpp
)
AddTests()
