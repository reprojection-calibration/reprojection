set(LIBRARY_NAME "caching")

find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)

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
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/hashing.test.cpp
        src/serialize.test.cpp
        test/cache_keys.test.cpp
)
AddTests()
