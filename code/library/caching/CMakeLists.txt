set(LIBRARY_NAME "caching")

set(SRC_FILES
        src/cache_keys.cpp
        src/hashing.cpp
        src/serialize.cpp
)
set(INCLUDE_DIRECTORIES
        ${OPENSSL_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OPENSSL_LIBRARIES}
        Eigen3::Eigen
        types
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        src/hashing.test.cpp
        src/serialize.test.cpp
        test/cache_keys.test.cpp
)
AddTests()
