set(LIBRARY_NAME "config")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus REQUIRED)

set(SRC_FILES
        src/ceres_solver_options.cpp
        src/load_configuration.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        tomlplusplus::tomlplusplus
        types
)
AddLibrary()

# TODO(Jack): Currently this only generates once at configure time. If a user changes the config file they need to rerun
# the cmake configuration to make sure the file gets copied over again. It might be possible to make this run on every
# build with "add_custom_target" and a dependency link. However this is for future work! If we find that cached copies
# are painful, also relevant for the sql/protobuf definitions, we should solve this really.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config.toml
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(TESTS
        src/ceres_solver_options.test.cpp
        src/ceres_enum_parsing.test.cpp
        test/load_configuration.test.cpp
)
AddTests()