macro(AddTests)
    foreach (TEST IN LISTS TESTS)
        get_filename_component(TEST_NAME ${TEST} NAME_WLE)
        set(TEST_NAME ${LIBRARY_NAME}.${TEST_NAME})

        add_executable(${TEST_NAME} ${TEST})
        target_include_directories(${TEST_NAME} PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
                ${INCLUDE_DIRECTORIES}
        )
        target_link_libraries(${TEST_NAME} PRIVATE
                GTest::gtest_main
                ${LIBRARY_NAME}
                ${LINK_LIBRARIES}
        )

        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            target_compile_options(${TEST_NAME} PRIVATE --coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif ()

        gtest_discover_tests(${TEST_NAME})
    endforeach ()
endmacro()