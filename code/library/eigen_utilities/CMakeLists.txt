set(LIBRARY_NAME "eigen_utilities")

set(SRC_FILES
        src/camera.cpp
        src/grid.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        types_internal
)
AddLibrary()

set(TESTS
        test/grid.test.cpp
)
AddTests()