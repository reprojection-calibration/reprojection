#pragma once

#include <sqlite3.h>
#include <string>

namespace reprojection::database {

enum class SqliteFlag {
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE
};

struct Sqlite3Tools {
    [[nodiscard]] static bool Execute(std::string const& sql_statement, sqlite3* const db);
};

}  // namespace reprojection::database
