#include "database/database_remove.hpp"

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "statement_executor.hpp"

namespace reprojection::database {

void RemoveFromDb(SqlitePtr const db, std::string_view entity_id, CalibrationStep const step) {
    auto const binder{[step, entity_id](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, ToString(step));
        Sqlite3Tools::Bind(stmt, 2, entity_id);
    }};

    ExecuteStatement(sql_statements::calibration_steps_delete, binder, db);
}

}  // namespace reprojection::database