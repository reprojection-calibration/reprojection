set(LIBRARY_NAME "config")

set(SRC_FILES
        src/config_loading.cpp
        src/config_parse.cpp
        src/parsing_helpers.cpp
        src/parsing_helpers3.cpp
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
        src/config3.test.cpp
        src/parsing_helpers.test.cpp
        src/parsing_helpers3.test.cpp
        test/config_loading.test.cpp
        test/config_parse.test.cpp
)
AddTests()