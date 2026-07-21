#pragma once

#include <sqlite3.h>

#include <stdexcept>

namespace reprojection::database {

class SqliteException : public std::runtime_error {
   public:
    SqliteException(sqlite3* const db, std::string_view sql);

    SqliteException(sqlite3* const db, sqlite3_stmt* const stmt);

    SqliteException(sqlite3* const db);

    SqliteException(sqlite3_stmt* const stmt);

   private:
    static std::string FormatMessage(sqlite3* const db, std::string_view sql);

    static std::string FormatMessage(sqlite3* const db, sqlite3_stmt* const stmt);

    static std::string FormatMessage(sqlite3* const db);

    static std::string FormatMessage(sqlite3_stmt* const stmt);

    static std::string Indent(std::string_view text);
};

}  // namespace reprojection::database