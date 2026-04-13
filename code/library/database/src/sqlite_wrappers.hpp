#pragma once

#include <sqlite3.h>

#include "types/io.hpp"

namespace reprojection::database {

struct SqlStatement {
    SqlStatement(SqlitePtr const& db, char const* const sql);

    ~SqlStatement();

    sqlite3_stmt* stmt{nullptr};
};

struct SqlTransaction {
    explicit SqlTransaction(SqlitePtr const& db);

    ~SqlTransaction();

   private:
    // TODO(Jack): This is an ugly reference semantics where we have a const ref to an object that we do not own. But
    // for the semantics of a lock (which the sql transaction basically is) maybe this is acceptable?
    SqlitePtr const& db_;
};

}  // namespace reprojection::database
