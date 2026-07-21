#pragma once

#include "database/sqlite_exception.hpp"

namespace reprojection::database {

// TODO(Jack): Add nullptr checks!
SqliteException::SqliteException(sqlite3* const db, std::string_view sql)
    : std::runtime_error(FormatMessage(db, sql)) {}

SqliteException::SqliteException(sqlite3* const db, sqlite3_stmt* const stmt)
    : std::runtime_error(FormatMessage(db, stmt)) {}

SqliteException::SqliteException(sqlite3* const db) : std::runtime_error(FormatMessage(db)) {}

SqliteException::SqliteException(sqlite3_stmt* const stmt) : std::runtime_error(FormatMessage(stmt)) {}

std::string SqliteException::FormatMessage(sqlite3* const db, std::string_view sql) {
    return "\n[SQLite Exception]\n"
           "----------------------------------------\n"
           "SQL Query:\n" +
           Indent(sql) + "\n" + "Error Code : " + std::to_string(sqlite3_errcode(db)) + "\n" +
           "Error Msg  : " + std::string(sqlite3_errmsg(db)) + "\n" + "----------------------------------------";
}

std::string SqliteException::FormatMessage(sqlite3* const db, sqlite3_stmt* const stmt) {
    // TODO(Jack): Add RAII handler for expanded?
    char* const expanded{sqlite3_expanded_sql(stmt)};
    std::string const sql{expanded != nullptr ? std::string{expanded} : std::string{sqlite3_sql(stmt)}};
    sqlite3_free(expanded);

    return FormatMessage(db, sql);
}

std::string SqliteException::FormatMessage(sqlite3* const db) {
    // TODO(Jack): Can we combine this with the full error message somehow so we do not need to copy and paste the
    // formatting twice?
    return "\n[SQLite Exception]\n"
           "----------------------------------------\n"
           "Error Code : " +
           std::to_string(sqlite3_errcode(db)) + "\n" + "Error Msg  : " + std::string(sqlite3_errmsg(db)) + "\n" +
           "----------------------------------------";
}

std::string SqliteException::FormatMessage(sqlite3_stmt* const stmt) {
    // TODO(Jack): Add RAII handler for expanded?
    char* const expanded{sqlite3_expanded_sql(stmt)};
    std::string const sql{expanded != nullptr ? std::string{expanded} : std::string{sqlite3_sql(stmt)}};
    sqlite3_free(expanded);

    // TODO(Jack): Can we combine this with the full error message somehow so we do not need to copy and paste the
    // formatting twice?
    return "\n[SQLite Exception]\n"
           "----------------------------------------\n"
           "SQL Query:\n" +
           Indent(sql) + "\n" + "----------------------------------------";
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