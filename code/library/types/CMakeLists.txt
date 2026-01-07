set(LIBRARY_NAME "types")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Header only package, therefore "INTERFACE" and we do not use the default cmake macros that we use for all the others.
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        ${OpenCV_INCLUDE_DIRS}
)

# WARN(Jack): For some reason once we included the opencv include dirs about we started getting compiler errors about an
# enum conversion at some place inside the opencv source code. Therefore we suppress that error for this target. I
# cannot say if this is the real problem or this is the real solution yet.
target_compile_options(${LIBRARY_NAME} INTERFACE
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>
)