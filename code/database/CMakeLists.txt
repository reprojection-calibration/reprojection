set(LIBRARY_NAME "database")

find_package(OpenCV REQUIRED)
find_package(SQLite3)

set(SRC_FILES
        src/calibration_database.cpp
        src/sqlite3_helpers.cpp
)
set(INCLUDE_DIRECTORIES
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        SQLite::SQLite3
)
AddLibrary()

set(TESTS
        src/sql.test.cpp
        src/sqlite3_helpers.test.cpp
        test/calibration_database.test.cpp
)
AddTests()