set(LIBRARY_NAME "testing_utilities")

file(READ "../../test_data/calibration_config.toml" contents)
set(TOML_CONFIG "\nstatic constexpr std::string_view minimum_config = R\"toml(\n${contents}\n)toml\";\n")
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/testing_utilities/minimum_config.hpp.in
        ${CMAKE_CURRENT_SOURCE_DIR}/include/testing_utilities/generated/minimum_config.hpp
        @ONLY
)

set(SRC_FILES
        src/temporary_file.cpp
)
set(INSTALL_SO OFF)
AddLibrary()

set(TESTS
        test/temporary_file.test.cpp
)
AddTests()