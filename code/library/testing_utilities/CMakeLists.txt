set(LIBRARY_NAME "testing_utilities")

file(READ "../../test_data/calibration_config.toml" contents)
set(TOML_CONFIG "static constexpr std::string_view minimum_config = R\"toml(${contents})toml\";")
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/testing_utilities/minimum_config.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/include/testing_utilities/generated/minimum_config.hpp
        @ONLY
)


set(SRC_FILES
        src/temporary_file.cpp
)
set(INSTALL_SO OFF)
AddLibrary()

target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

set(TESTS
        test/temporary_file.test.cpp
)
AddTests()

message(STATUS "binary dir = ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "includes = ${INCLUDE_DIRECTORIES}")