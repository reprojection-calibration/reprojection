set(LIBRARY_NAME "testing_utilities")

set(SRC_FILES
        src/temporary_file.cpp
)
set(INSTALL_SO OFF)
AddLibrary()

set(TESTS
        test/temporary_file.test.cpp
)
AddTests()