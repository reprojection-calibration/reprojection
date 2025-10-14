cmake_minimum_required(VERSION 3.28)

set(LIBRARY_NAME "geometry")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/lie.cpp
)
add_library(${LIBRARY_NAME} SHARED
        ${SRC_FILES}
)
target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
)
target_link_libraries(${LIBRARY_NAME} PUBLIC
        Eigen3::Eigen
)


enable_testing()
include(GoogleTest)

set(TESTS
        src/lie.test.cpp
)
foreach (TEST IN LISTS TESTS)
    get_filename_component(TEST_NAME ${TEST} NAME_WLE)
    add_executable(${TEST_NAME} ${TEST})
    target_link_libraries(${TEST_NAME}
            GTest::gtest_main
            ${LIBRARY_NAME}
    )
    gtest_discover_tests(${TEST_NAME})
endforeach ()
