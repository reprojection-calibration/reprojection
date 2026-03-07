#pragma once

#include <sqlite3.h>

#include <string_view>

#include "sqlite3_helpers.hpp"
#include "sqlite_wrappers.hpp"

namespace reprojection::database {

// TODO(Jack): Add override without a binder if we use that case not just in testing.
template <typename Binder, typename RowFunc>
void ExecuteQuery(sqlite3* const db, std::string_view sql, Binder&& binder, RowFunc&& on_row) {
    SqlStatement statement{db, std::string(sql).data()};

    try {
        // NOTE(Jack): If the sql query statement does not use any dynamic binding, then we do not need to call the
        // binder. Therefore, this code lets the user pass in a nullptr for the binder, and it will then not execute any
        // binding call. If this is actually possible for people to understand is not clear at this time :)
        if constexpr (not std::is_same_v<std::decay_t<Binder>, std::nullptr_t>) {
            binder(statement.stmt);
        }
    } catch (...) {                      // LCOV_EXCL_LINE
        throw SqliteException(db, sql);  // LCOV_EXCL_LINE
    }

    while (Sqlite3Tools::StepRow(statement.stmt)) {
        on_row(statement.stmt);
    }
}

}  // namespace reprojection::database