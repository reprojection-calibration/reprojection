#pragma once

#include <sqlite3.h>

#include <string_view>

#include "database/sqlite_wrappers.hpp"

#include "statement_executor.hpp"

namespace reprojection::database {

template <typename Container, typename Binder>
void BatchInsert(std::string_view sql, Container const& data, Binder&& binder, sqlite3* db) {
    for (SqlTransaction const transaction{db}; auto const& data_i : data) {
        ExecuteStatement(sql, [&](sqlite3_stmt* stmt) { binder(stmt, data_i); }, db);
    }
}

}  // namespace reprojection::database