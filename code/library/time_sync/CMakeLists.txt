set(LIBRARY_NAME "time_sync")

set(SRC_FILES
        src/time_sync.cpp
)

AddLibrary()


set(TESTS
        test/time_sync.test.cpp
)
AddTests()
