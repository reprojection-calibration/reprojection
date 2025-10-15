macro(AddTests)
    if (CODE_COVERAGE)
        target_compile_options(${LIBRARY_NAME} PRIVATE --coverage -O0 -g)
        target_link_options(${LIBRARY_NAME} PRIVATE --coverage)
    endif ()


    include(GoogleTest)

    foreach (TEST IN LISTS TESTS)
        get_filename_component(TEST_NAME ${TEST} NAME_WLE)
        set(TEST_NAME ${LIBRARY_NAME}.${TEST_NAME})

        add_executable(${TEST_NAME} ${TEST})
        target_link_libraries(${TEST_NAME}
                GTest::gtest_main
                ${LIBRARY_NAME}
        )

        if (CODE_COVERAGE)
            target_compile_options(${TEST_NAME} PRIVATE --coverage -O0 -g)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif ()

        gtest_discover_tests(${TEST_NAME})
    endforeach ()
endmacro()