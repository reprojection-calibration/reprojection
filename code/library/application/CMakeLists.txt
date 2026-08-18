set(LIBRARY_NAME "application")

set(SRC_FILES
        src/cli_utils.cpp
        src/io.cpp
        src/reprojection_calibration.cpp
)

set(LINK_LIBRARIES
        SQLite3::SQLite3
        config
        database
        logging
        steps
        types_internal
        video_capture
)
set(PUBLIC_LINK_LIBRARIES
        ${OpenCV_LIBS}
        tomlplusplus::tomlplusplus
        types
)

set(INSTALL_HEADERS ON)
AddLibrary()

set(TEST_LINK_LIBRARIES
        hashing
        testing_mocks
        testing_utilities
)
set(TESTS
        src/io.test.cpp
        test/cli_utils.test.cpp
        test/reprojection_calibration.test.cpp
)
AddTests()



set(EXAMPLES
        examples/calibration.cpp
)
AddExamples()