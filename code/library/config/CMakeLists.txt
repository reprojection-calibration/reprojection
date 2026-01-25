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

set(TESTS
        src/ceres_solver_options.test.cpp
        src/ceres_enum_parsing.test.cpp
        test/load_configuration.test.cpp
)
AddTests()