set(LIBRARY_NAME "testing_mocks")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/mvg_generator.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        geometry
        spline
)
AddLibrary()

set(TESTS
        test/mvg_generator.test.cpp
)
AddTests()