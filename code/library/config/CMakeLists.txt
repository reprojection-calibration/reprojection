set(LIBRARY_NAME "config")

set(SRC_FILES
        src/config2.cpp
        src/config_loading.cpp
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        logging
        tomlplusplus::tomlplusplus
        types
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        test/config_loading.test.cpp
        src/parsing_helpers.test.cpp
        test/config2.test.cpp
)
AddTests()