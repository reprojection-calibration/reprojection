#pragma once

#include <filesystem>
#include <memory>

#include "types/io.hpp"

namespace reprojection::database {

namespace fs = std::filesystem;

SqlitePtr OpenCalibrationDatabase(fs::path const& db_path, bool const create, bool const read_only = false);

// TODO(Jack): When I first did the smart pointer database refactor I started with a unique pointer. And in the unique
// template definition you can directly define and add the deleter. For the shared pointer you cannot specify the
// deleter directly in the using definition. Therefore we define it here and need to remember to pass it to each place
// where we create the database smart pointer. There has be a better way to do this...
struct SqliteDeleter {
    void operator()(sqlite3* const db) const {
        if (db) {
            sqlite3_close(db);
        }
    }
};

}  // namespace reprojection::database