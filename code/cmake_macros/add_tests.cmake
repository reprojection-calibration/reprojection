macro(AddTests)
    enable_testing()
    include(GoogleTest)

    foreach (TEST IN LISTS TESTS)
        get_filename_component(TEST_NAME ${TEST} NAME_WLE)
        add_executable(${TEST_NAME} ${TEST})
        target_link_libraries(${TEST_NAME}
                GTest::gtest_main
                ${LIBRARY_NAME}
        )
        gtest_discover_tests(${TEST_NAME})
    endforeach ()
endmacro()