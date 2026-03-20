cmake_minimum_required(VERSION 3.10)
project(reprojection)

include(../cmake_macros/add_library.cmake)
include(../cmake_macros/add_tests.cmake)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})
set(SRC_FILES
        src/reprojection.cpp
)
set(INCLUDE_DIRECTORIES
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
)
AddLibrary()

find_package(GTest REQUIRED)
enable_testing()


set(TESTS
        test/reprojection.test.cpp
)
AddTests()

set(EXAMPLES
        examples/reprojection_app.cpp
)
foreach (EXAMPLE IN LISTS EXAMPLES)
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WLE)
    set(EXAMPLE_NAME ${LIBRARY_NAME}.${EXAMPLE_NAME})

    add_executable(${EXAMPLE_NAME} ${EXAMPLE})
    target_include_directories(${EXAMPLE_NAME} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
            ${INCLUDE_DIRECTORIES}
    )
    target_link_libraries(${EXAMPLE_NAME}
            ${LIBRARY_NAME}
            ${LINK_LIBRARIES}
    )
endforeach ()

