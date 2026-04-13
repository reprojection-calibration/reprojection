#pragma once

#include <sqlite3.h>

#include <string_view>

#include "database/calibration_database.hpp"

#include "sqlite3_helpers.hpp"
#include "sqlite_wrappers.hpp"

namespace reprojection::database {

template <typename Binder>
void ExecuteStatement(std::string_view sql, Binder&& binder, SqlitePtr const db) {
    SqlStatement statement{db, std::string(sql).c_str()};

    try {
        binder(statement.stmt);
    } catch (...) {
        // TODO(Jack): It think it can very well be that any error thrown from bind is actually not 100% sqlite related,
        //  but actually due a error in the user code. Therefore it might be a mistake here to throw away the thrown
        //  error and replace it here with a database centric error. Think about also throwing the original error too!
        throw SqliteException(db, sql);
    }

    if (sqlite3_step(statement.stmt) != SQLITE_DONE) {
        throw SqliteException(db, sql);
    }
}

// Used for cases that do not require dynamic binding - passes an empty lambda which is a no-op.
inline void ExecuteStatement(std::string_view sql, SqlitePtr const db) {
    ExecuteStatement(sql, [](sqlite3_stmt*) {}, db);
}

// TODO(Jack): Can we use concepts here to enforce some properties on Container and Binder?
template <typename Container, typename Binder>
void BatchExecuteStatement(std::string_view sql, Container const& data, Binder&& binder, SqlitePtr const db) {
    for (SqlTransaction const transaction{db}; auto const& data_i : data) {
        ExecuteStatement(sql, [&](sqlite3_stmt* stmt) { binder(stmt, data_i); }, db);
    }
}

}  // namespace reprojection::database