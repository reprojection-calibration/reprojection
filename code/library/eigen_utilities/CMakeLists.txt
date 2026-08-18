set(LIBRARY_NAME "eigen_utilities")

set(SRC_FILES
        src/camera.cpp
        src/grid.cpp
        src/statistics.cpp
)
set(PRIVATE_LINK_LIBRARIES
        types_internal
)
AddLibrary()

set(TESTS
        test/grid.test.cpp
        test/statistics.test.cpp
)
AddTests()