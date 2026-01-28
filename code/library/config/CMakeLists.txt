set(LIBRARY_NAME "config")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus REQUIRED)

set(SRC_FILES
        src/enum_string_converters.cpp
        src/load_config_file.cpp
        src/parse_ceres_solver_options.cpp
        src/toml_helpers.cpp
        src/validate_target_config.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        testing_utilities
        tomlplusplus::tomlplusplus
        types
)
AddLibrary()

set(TESTS
        src/enum_string_converters.test.cpp
        src/parse_ceres_solver_options.test.cpp
        src/toml_helpers.test.cpp
        test/load_config_file.test.cpp
        test/validate_target_config.test.cpp
)
AddTests()