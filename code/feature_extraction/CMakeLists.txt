set(LIBRARY_NAME "feature-extraction")

# Taken from https://github.com/amy-tabb/apriltag-cmake-example/tree/master (did not include the link_directories() command here)
find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

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
        ${YAML_CPP_INCLUDE_DIR}
)
set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        types
        ${apriltag_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
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

# TODO(Jack): When we have multiple examples we can make this a macro like we did for the library and testing
set(EXAMPLES
        examples/webcam_demo.cpp
)
foreach (EXAMPLE IN LISTS EXAMPLES)
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WLE)
    set(EXAMPLE_NAME ${LIBRARY_NAME}.${EXAMPLE_NAME})

    add_executable(${EXAMPLE_NAME} ${EXAMPLE})
    target_link_libraries(${EXAMPLE_NAME}
            ${LIBRARY_NAME}
            ${LINK_LIBRARIES}
    )
endforeach ()


set(TESTS
        src/april_tag_cpp_wrapper.test.cpp
        src/target_extractors.test.cpp
        src/target_generators.test.cpp
        test/target_extraction.test.cpp
        src/utilities.test.cpp
)
AddTests()