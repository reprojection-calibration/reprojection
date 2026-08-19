set(LIBRARY_NAME "testing_utilities")

# WARN(Jack): You need to rerun the cmake generation if you want changes to the config file to make it into the library!
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

# NOTE(Jack): We cannot put generated code on the host filesystem because or docker philosophy says that we never put an
# artefact on host unless they ask for it (ex. a calibration database). Therefore the generated code here and for the
# generated sql code in the database package, gets placed in the build folder. Therefore we need to add special include
# instructions for just those two packages so the libraries are able to find the includes.
target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

set(TESTS
        test/temporary_file.test.cpp
)
AddTests()
