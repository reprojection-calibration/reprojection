set(LIBRARY_NAME "transforms")

set(SRC_FILES
        src/extrinsics.cpp
)
set(PRIVATE_LINK_LIBRARIES
        geometry
        types_internal
)
AddLibrary()

set(TESTS
        test/extrinsics.test.cpp
)
AddTests()