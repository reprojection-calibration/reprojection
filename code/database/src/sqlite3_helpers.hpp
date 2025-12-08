#pragma once

#include <sqlite3.h>

#include <iostream>

namespace reprojection::database {

enum class SqliteFlag {
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE
};

struct Sqlite3Tools {
    [[nodiscard]] static bool Execute(std::string const& sql_statement, sqlite3* const db) {
        char* errror_msg{0};
        int const code{sqlite3_exec(db, sql_statement.c_str(), nullptr, nullptr, &errror_msg)};

        if (code != static_cast<int>(SqliteFlag::Ok)) {
            std::cerr << "SQL error: " << errror_msg << std::endl;
            sqlite3_free(errror_msg);  // WARN(Jack): Violating RAII here! Should wrap errror_msg with class.

            return false;
        }

        return true;
    }
};

}  // namespace reprojection::database
