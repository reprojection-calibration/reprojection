set(LIBRARY_NAME "database")

find_package(SQLite3)

set(SRC_FILES
        src/database.cpp
)
set(LINK_LIBRARIES
        SQLite::SQLite3
)
AddLibrary()

set(TESTS
        test/database.test.cpp
)
AddTests()