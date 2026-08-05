#include "database/sqlite_exception.hpp"

namespace reprojection::database {

SqliteException::SqliteException(sqlite3* const db, std::string_view sql)
    : std::runtime_error(FormatMessage(db, sql)) {}

SqliteException::SqliteException(sqlite3* const db, sqlite3_stmt* const stmt)
    : std::runtime_error(FormatMessage(db, ExpandedSql(stmt))) {}

SqliteException::SqliteException(sqlite3* const db) : std::runtime_error(FormatMessage(db, std::nullopt)) {}

SqliteException::SqliteException(sqlite3_stmt* const stmt)
    : std::runtime_error(FormatMessage(nullptr, ExpandedSql(stmt))) {}

std::string SqliteException::FormatMessage(sqlite3* const db, std::optional<std::string_view> const sql) {
    std::string message{
        "\n[SQLite Exception]\n"
        "----------------------------------------\n"};

    if (sql) {
        message += "SQL Query:";
        message += Indent(*sql);
        message += '\n';
    }

    if (db != nullptr) {
        message += '\n';
        message += "Error Code : " + std::to_string(sqlite3_errcode(db)) + '\n';
        message += "Error Msg  : " + std::string{sqlite3_errmsg(db)} + '\n';
    }

    message += "----------------------------------------";

    return message;
}

std::string SqliteException::ExpandedSql(sqlite3_stmt* const stmt) {
    char* const expanded{sqlite3_expanded_sql(stmt)};
    std::string const sql{expanded != nullptr ? expanded : sqlite3_sql(stmt)};
    sqlite3_free(expanded);

    return sql;
}

std::string SqliteException::Indent(std::string_view text) {
    std::string result;
    result.reserve(text.size() + 16);

    result += "  ";
    for (char c : text) {
        result += c;
        if (c == '\n') {
            result += "  ";
        }
    }

    return result;
}

}  // namespace reprojection::database