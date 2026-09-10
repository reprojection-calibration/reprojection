set(LIBRARY_NAME "time_synchronization")

set(SRC_FILES
        src/time_synchronization.cpp
)

AddLibrary()


set(TESTS
        test/time_synchronization.test.cpp
)
AddTests()
