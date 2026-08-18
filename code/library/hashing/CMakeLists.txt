set(LIBRARY_NAME "hashing")

set(SRC_FILES
        src/hashing.cpp
        src/serialize.cpp
)
set(INCLUDE_DIRECTORIES
        ${OPENSSL_INCLUDE_DIRS}
)
set(PRIVATE_LINK_LIBRARIES
        ${OPENSSL_LIBRARIES}
        types_internal
)
set(PUBLIC_LINK_LIBRARIES
        config
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        test/hashing.test.cpp
        test/serialize.test.cpp
)
AddTests()
