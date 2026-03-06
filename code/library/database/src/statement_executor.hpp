#pragma once

#include <sqlite3.h>

#include <string_view>

#include "sqlite3_helpers.hpp"
#include "sqlite_wrappers.hpp"

namespace reprojection::database {

template <typename Binder>
void ExecuteStatement(std::string_view sql, Binder&& binder, sqlite3* db) {
    SqlStatement statement{db, std::string(sql).c_str()};

    try {
        binder(statement.stmt);
    } catch (...) {
        throw SqliteException(db, sql);
    }

    if (sqlite3_step(statement.stmt) != SQLITE_DONE) {
        throw SqliteException(db, sql);
    }
}

// Used for cases that do not require dynamic binding - passes an empty lambda which is a no-op.
inline void ExecuteStatement(std::string_view sql, sqlite3* db) {
    ExecuteStatement(sql, [](sqlite3_stmt*) {}, db);
}

// TODO(Jack): Can we use concepts here to enforce some properties on Container and Binder?
template <typename Container, typename Binder>
void BatchExecuteStatement(std::string_view sql, Container const& data, Binder&& binder, sqlite3* db) {
    for (SqlTransaction const transaction{db}; auto const& data_i : data) {
        ExecuteStatement(sql, [&](sqlite3_stmt* stmt) { binder(stmt, data_i); }, db);
    }
}

}  // namespace reprojection::database