set(LIBRARY_NAME "feature_extraction")

# Taken from https://github.com/amy-tabb/apriltag-cmake-example/tree/master (did not include the link_directories() command here)
find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tomlplusplus REQUIRED)

set(SRC_FILES
        src/april_tag_cpp_wrapper.cpp
        src/generated_apriltag_code/tagCustom36h11.c
        src/utilities.cpp
        src/target_extraction.cpp
        src/target_extractors.cpp
        src/target_generators.cpp
)
set(INCLUDE_DIRECTORIES
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        config
        Eigen3::Eigen
        eigen_utilities
        tomlplusplus::tomlplusplus
        types
        ${apriltag_LIBRARIES}
        ${OpenCV_LIBS}
)
AddLibrary()

# For inexplicable reasons, the docker build-stage build will fail if we do not include this link directory...
# Without this it works fine in CLion, but when we build it manually it gives us problems. I should note that I
# diverged from the original "tip" about using april grid in cmake found here (https://github.com/amy-tabb/apriltag-cmake-example/tree/master).
# Because I only wanted to use target specific commands, I omitted the link_directories() command and instead add
# the april grid libraries via a target_link_libraries() command to our library.
target_link_directories(${LIBRARY_NAME} PUBLIC
        ${apriltag_LIBDIR}
)

set(TESTS
        src/april_tag_cpp_wrapper.test.cpp
        src/target_extractors.test.cpp
        src/target_generators.test.cpp
        test/target_extraction.test.cpp
        src/utilities.test.cpp
)
AddTests()