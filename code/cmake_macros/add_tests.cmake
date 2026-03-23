macro(AddTests)
    foreach (TEST IN LISTS TESTS)
        get_filename_component(TEST_NAME ${TEST} NAME_WLE)
        set(TEST_NAME ${LIBRARY_NAME}.${TEST_NAME})

        add_executable(${TEST_NAME} ${TEST})
        target_include_directories(${TEST_NAME} PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
                ${INCLUDE_DIRECTORIES}
        )
        # NOTE(Jack): We do not want our shared libraries which we export polluted with dependencies that are only used
        # during library unit testing. Therefore we define TEST_LINK_LIBRARIES and only link them against the test
        # executables. This keeps the set of libraries and dependencies we need to export clean!
        target_link_libraries(${TEST_NAME} PRIVATE
                GTest::gtest_main
                ${LIBRARY_NAME}
                ${LINK_LIBRARIES}
                ${TEST_LINK_LIBRARIES}
        )

        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            target_compile_options(${TEST_NAME} PRIVATE --coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif ()

        gtest_discover_tests(${TEST_NAME})
    endforeach ()
endmacro()