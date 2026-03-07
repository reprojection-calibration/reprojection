#include "database/sensor_data_interface_getters.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "query_runner.hpp"
#include "serialization.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// NOTE(Jack): The core sql handling logic here is very similar to the ImageStreamer class, but there are enough
// differences that we cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
// NOTE(Jack): See notes above to understand why we suppress code coverage.
CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name) {
    CameraMeasurements data;

    ExecuteQuery(  // LCOV_EXCL_LINE
        database->db, sql_statements::extracted_targets_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&data, sensor_name](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{Sqlite3Tools::SqliteBlob(stmt, 1)};
            protobuf_serialization::ExtractedTargetProto serialized;
            serialized.ParseFromArray(std::data(blob), static_cast<int>(std::size(blob)));

            auto const deserialized{Deserialize(serialized)};
            if (not deserialized) {
                throw std::runtime_error("Deserialize(ExtractedTargetProto) failed for " +  // LCOV_EXCL_LINE
                                         std::string(sensor_name));                         // LCOV_EXCL_LINE
            }

            data.insert({timestamp_ns, deserialized.value()});
        });

    return data;
}  // LCOV_EXCL_LINE

std::optional<std::string> ReadCacheKey(std::shared_ptr<CalibrationDatabase const> const database,
                                        CalibrationStep const step_name, std::string_view sensor_name) {
    std::optional<std::string> cache_key;

    ExecuteQuery(
        database->db, sql_statements::calibration_steps_select,
        [step_name, sensor_name](sqlite3_stmt* const stmt) {
            Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
            Sqlite3Tools::Bind(stmt, 2, sensor_name);
        },
        [&cache_key](sqlite3_stmt* const stmt) {
            if (const unsigned char* text = sqlite3_column_text(stmt, 0)) {
                cache_key = std::string(reinterpret_cast<char const*>(text));
            } else {
                cache_key = std::nullopt;
            }
        });

    return cache_key;
}

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name) {
    ImuMeasurements data;

    ExecuteQuery(  // LCOV_EXCL_LINE
        database->db, sql_statements::imu_data_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            double const omega_x{sqlite3_column_double(stmt, 1)};
            double const omega_y{sqlite3_column_double(stmt, 2)};
            double const omega_z{sqlite3_column_double(stmt, 3)};

            double const ax{sqlite3_column_double(stmt, 4)};
            double const ay{sqlite3_column_double(stmt, 5)};
            double const az{sqlite3_column_double(stmt, 6)};

            data.insert(ImuMeasurement{timestamp_ns, {{omega_x, omega_y, omega_z}, {ax, ay, az}}});
        });

    return data;
}  // LCOV_EXCL_LINE

};  // namespace reprojection::database