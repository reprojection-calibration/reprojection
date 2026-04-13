#pragma once

#include <sqlite3.h>

#include <memory>
#include <string>

namespace reprojection::database {

struct SqliteDeleter {
    void operator()(sqlite3* const db) const {
        if (db) {
            sqlite3_close(db);
        }
    }
};

using SqlitePtr = std::unique_ptr<sqlite3, SqliteDeleter>;

SqlitePtr OpenCalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false);


}  // namespace reprojection::database