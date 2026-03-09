set(LIBRARY_NAME "application")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/application.cpp
        src/steps.cpp
)

set(LINK_LIBRARIES
        caching
        calibration
        database
        Eigen3::Eigen
        optimization
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/step_runner.test.cpp
        test/application.test.cpp
)
AddTests()
