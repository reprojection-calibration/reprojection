set(LIBRARY_NAME "config")

set(SRC_FILES
        src/config_loading.cpp
        src/config_parsing.cpp
        src/enum_string_converters.cpp
        src/parse_ceres_solver_options.cpp
        src/toml_helpers.cpp
        src/config_validation.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        tomlplusplus::tomlplusplus
        types
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        src/enum_string_converters.test.cpp
        src/parse_ceres_solver_options.test.cpp
        src/toml_helpers.test.cpp
        test/config_loading.test.cpp
        test/config_parsing.test.cpp
        test/validate_config.test.cpp
        test/validate_target_config.test.cpp
)
AddTests()