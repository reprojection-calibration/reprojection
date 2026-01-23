set(LIBRARY_NAME "config")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus REQUIRED)

set(SRC_FILES
        src/ceres_solver_params.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        tomlplusplus::tomlplusplus
        types
)
AddLibrary()

set(TESTS
        src/enum_parsing.test.cpp
        test/ceres_solver_params.test.cpp
)
AddTests()