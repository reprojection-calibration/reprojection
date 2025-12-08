set(LIBRARY_NAME "database")

find_package(SQLite3)

set(SRC_FILES
        src/database.cpp
        src/sqlite3_helpers.cpp
)
set(LINK_LIBRARIES
        SQLite::SQLite3
)
AddLibrary()

set(TESTS
        src/sqlite3_helpers.test.cpp
        test/database.test.cpp
)
AddTests()