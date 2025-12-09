#include "database/calibration_database.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "sql.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

CalibrationDatabase::CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only) {
    if (create and read_only) {
        throw std::runtime_error(
            "You requested to open a database object with both options 'create' and 'read_only' true. This is "
            "an invalid combination as creating a database requires writing to it!");
    }

    // TODO(Jack): Consider using sqlite3_errcode for better terminal output https://sqlite.org/c3ref/errcode.html
    int code;
    if (create) {
        // WARN(Jack): Should it be an error if create is true and the database already exists. Is that a problem?
        code = sqlite3_open_v2(db_path.c_str(), &db,
                               static_cast<int>(SqliteFlag::OpenReadWrite) | static_cast<int>(SqliteFlag::OpenCreate),
                               nullptr);
    } else if (read_only) {
        code = sqlite3_open_v2(db_path.c_str(), &db, static_cast<int>(SqliteFlag::OpenReadOnly), nullptr);
    } else {
        code = sqlite3_open_v2(db_path.c_str(), &db, static_cast<int>(SqliteFlag::OpenReadWrite), nullptr);
    }

    if (code != 0) {
        sqlite3_close(db);
        throw std::runtime_error("Attempted to open database at path - " + db_path + " - but was unsuccessful");
    }

    // WARN(Jack): Is there any circumstance under which the data table creation might fail, and casting to void here
    // instead of explicitly handling the status makes sense?
    static_cast<void>(Sqlite3Tools::Execute(imu_table_sql, db));
    static_cast<void>(Sqlite3Tools::Execute(images_table_sql, db));
}

CalibrationDatabase::~CalibrationDatabase() { sqlite3_close(db); }

};  // namespace reprojection::database