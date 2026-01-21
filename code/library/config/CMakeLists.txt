set(LIBRARY_NAME "config")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus REQUIRED)

set(SRC_FILES
        src/xxx.cpp
)
set(LINK_LIBRARIES
        Ceres::ceres
        Eigen3::Eigen
        tomlplusplus::tomlplusplus
        types
)
AddLibrary()

set(TESTS
        test/xxx.test.cpp
)
AddTests()