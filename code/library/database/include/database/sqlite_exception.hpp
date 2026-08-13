#pragma once

#include <sqlite3.h>

#include <optional>
#include <stdexcept>

namespace reprojection::database {

class SqliteException : public std::runtime_error {
   public:
    SqliteException(sqlite3* db, std::string_view sql);

    SqliteException(sqlite3* db, sqlite3_stmt* stmt);

    explicit SqliteException(sqlite3* db);

    explicit SqliteException(sqlite3_stmt* stmt);

   private:
    static std::string FormatMessage(sqlite3* db, std::optional<std::string_view> sql);

    static std::string ExpandedSql(sqlite3_stmt* stmt);
    static std::string Indent(std::string_view text);
};

}  // namespace reprojection::database