set(LIBRARY_NAME "logging")

set(SRC_FILES
        src/formatting.cpp
        src/logging.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        spdlog::spdlog
        tomlplusplus::tomlplusplus
)
AddLibrary()


set(TESTS
        test/logging.test.cpp
)
AddTests()