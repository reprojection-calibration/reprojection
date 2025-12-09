#pragma once

#include <sqlite3.h>

#include <functional>
#include <string>

namespace reprojection::database {

enum class SqliteFlag {
    Done = SQLITE_DONE,
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE
};

struct Sqlite3Tools {
    using CallbackType = int (*)(void*, int, char**, char**);

    [[nodiscard]] static bool Execute(std::string const& sql_statement, sqlite3* const db,
                                      CallbackType callback = nullptr, void* data_structure = nullptr);
};

}  // namespace reprojection::database
