set(LIBRARY_NAME "hashing")

set(SRC_FILES
        src/hashing.cpp
        src/serialize.cpp
)
set(INCLUDE_DIRECTORIES
        ${OPENSSL_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OPENSSL_LIBRARIES}
        Eigen3::Eigen
        config
        tomlplusplus::tomlplusplus # REQUIRED BECAUSE OF CONFIG DEP - SHOULD TRY TO REMOVE
        types_internal
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
