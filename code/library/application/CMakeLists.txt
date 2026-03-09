set(LIBRARY_NAME "application")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/application.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        types
)
AddLibrary()

set(TESTS
        test/application.test.cpp
)
AddTests()
