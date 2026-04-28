set(LIBRARY_NAME "eigen_utilities")

set(SRC_FILES
        src/camera.cpp
        src/grid.cpp
        src/statistics.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        types_internal
)
AddLibrary()

set(TESTS
        test/grid.test.cpp
        test/statistics.test.cpp
)
AddTests()