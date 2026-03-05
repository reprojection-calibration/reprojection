#include "database/sensor_data_interface_updater.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

#include "database/sqlite_wrappers.hpp"
// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// TODO(Jack): Refactor to a specific cache table that holds the cache keys specific for each sensor.
void UpdateCalibrationStep(std::string_view step_name, std::string_view cache_key,
                           std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::calibration_steps_update};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, cache_key);
        Sqlite3Tools::Bind(statement.stmt, 2, step_name);
    } catch (std::runtime_error const& e) {                                          // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(                                   // LCOV_EXCL_LINE
            ErrorMessage("UpdateCalibrationStep()", SqliteErrorCode::FailedBinding,  // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database->db)))));               // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        throw std::runtime_error(ErrorMessage("UpdateCalibrationStep()", SqliteErrorCode::FailedStep,
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

}  // namespace reprojection::database