#pragma once

#include <sqlite3.h>

namespace reprojection::database {

struct SqlStatement {
    SqlStatement(sqlite3* const db, char const* const sql);

    ~SqlStatement();

    sqlite3_stmt* stmt{nullptr};
};

}  // namespace reprojection::database
