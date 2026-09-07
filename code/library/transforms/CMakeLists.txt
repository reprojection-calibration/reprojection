set(LIBRARY_NAME "transforms")

set(SRC_FILES
        src/extrinsic.cpp
)
set(PRIVATE_LINK_LIBRARIES
        geometry
        types_internal
)
AddLibrary()

set(TESTS
        test/extrinsic.test.cpp
)
AddTests()