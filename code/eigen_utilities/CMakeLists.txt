set(LIBRARY_NAME "eigen_utilities")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/grid.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
)
AddLibrary()

set(TESTS
        test/grid.test.cpp
)
AddTests()