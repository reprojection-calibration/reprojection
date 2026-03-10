#include "database/database_remove.hpp"

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "statement_executor.hpp"

namespace reprojection::database {

void RemoveFromDb(CalibrationStep const step, std::string_view sensor_name, DbPtr const db) {
    auto const binder{[step, sensor_name](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, ToString(step));
        Sqlite3Tools::Bind(stmt, 2, sensor_name);
    }};

    ExecuteStatement(sql_statements::calibration_steps_delete, binder, db->db);
}

}  // namespace reprojection::database