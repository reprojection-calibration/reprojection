#pragma once

#include <sqlite3.h>

namespace reprojection::database {

struct SqlStatement {
    SqlStatement(sqlite3* const db, char const* const sql);

    ~SqlStatement();

    sqlite3_stmt* stmt{nullptr};
};

struct SqlTransaction {
    SqlTransaction(sqlite3* const db);

    ~SqlTransaction();

   private:
    // TODO(Jack): Should this be a smart pointer?
    sqlite3* db_;
};

}  // namespace reprojection::database
