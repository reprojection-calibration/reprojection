set(LIBRARY_NAME "types_internal")

# TODO(Jack): For some reason as an interface package we need to add it to every single library we want to use it - the
# transitive dependency mechanism of a PUBLIC_LINK_LIBRARY does not work here, or at least it did not on my first try.
# Is there a more eloquent way to solve this?

# Header only package, therefore "INTERFACE" and we do not use the default cmake macros that we use for all the others.
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME} INTERFACE
        Eigen3::Eigen
)

# WARN(Jack): For some reason once we included the opencv include dirs about we started getting compiler errors about an
# enum conversion at some place inside the opencv source code. Therefore we suppress that error for this target. I
# cannot say if this is the real problem or this is the real solution yet.
target_compile_options(${LIBRARY_NAME} INTERFACE
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>
)