#include "database/sensor_data_interface_getters.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

#include "database/sqlite_wrappers.hpp"
// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "serialization.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// NOTE(Jack): The core sql handling logic here is very similar to the ImageStreamer class, but there are enough
// differences that we cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
// NOTE(Jack): See notes above to understand why we suppress code coverage.
CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name) {
    SqlStatement const statement{database->db, sql_statements::extracted_targets_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, std::string(sensor_name));
    } catch (std::runtime_error const& e) {                             // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(                      // LCOV_EXCL_LINE
            ErrorMessage("GetExtractedTargetData()", sensor_name, 0,    // LCOV_EXCL_LINE
                         SqliteErrorCode::FailedBinding,                // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database->db)))));  // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    CameraMeasurements targets;
    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            throw std::runtime_error(ErrorMessage(                                         // LCOV_EXCL_LINE
                "GetExtractedTargetData()", sensor_name, 0,                                // LCOV_EXCL_LINE
                SqliteErrorCode::FailedStep, std::string(sqlite3_errmsg(database->db))));  // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be more defensive here and first check that column text is not returning a nullptr or
        // other bad output? Also happens like this in the image streamer and possibly other places.
        uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(statement.stmt, 0)))};

        uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(statement.stmt, 1))};
        int const blob_size{sqlite3_column_bytes(statement.stmt, 1)};
        if (not blob or blob_size <= 0) {
            throw std::runtime_error("GetExtractedTargetData() blob reading failed for sensor: " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name) +                                     // LCOV_EXCL_LINE
                                     " at timestamp_ns: " + std::to_string(timestamp_ns));          // LCOV_EXCL_LINE
        }

        std::vector<uchar> const buffer(blob, blob + blob_size);
        protobuf_serialization::ExtractedTargetProto serialized;
        serialized.ParseFromArray(buffer.data(), buffer.size());

        auto const deserialized{Deserialize(serialized)};
        if (not deserialized.has_value()) {
            throw std::runtime_error("GetExtractedTargetData() Deserialize() failed for sensor: " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name) +                                      // LCOV_EXCL_LINE
                                     " at timestamp_ns: " + std::to_string(timestamp_ns));           // LCOV_EXCL_LINE
        }

        targets.insert({timestamp_ns, deserialized.value()});
    }

    return targets;
}

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name) {
    SqlStatement const statement{database->db, sql_statements::imu_data_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, std::string(sensor_name));
    } catch (...) {                                                     // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(                      // LCOV_EXCL_LINE
            ErrorMessage("GetImuData()", sensor_name, 0,                // LCOV_EXCL_LINE
                         SqliteErrorCode::FailedBinding,                // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database->db)))));  // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    ImuMeasurements data;
    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            throw std::runtime_error(ErrorMessage("GetImuData()", sensor_name, 0,               // LCOV_EXCL_LINE
                                                  SqliteErrorCode::FailedStep,                  // LCOV_EXCL_LINE
                                                  std::string(sqlite3_errmsg(database->db))));  // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be doing any error checking here while reading the columns?
        uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(statement.stmt, 0))};  // Warn cast!
        double const omega_x{sqlite3_column_double(statement.stmt, 1)};
        double const omega_y{sqlite3_column_double(statement.stmt, 2)};
        double const omega_z{sqlite3_column_double(statement.stmt, 3)};
        double const ax{sqlite3_column_double(statement.stmt, 4)};
        double const ay{sqlite3_column_double(statement.stmt, 5)};
        double const az{sqlite3_column_double(statement.stmt, 6)};

        data.insert(ImuMeasurement{timestamp_ns, {{omega_x, omega_y, omega_z}, {ax, ay, az}}});
    }

    return data;
}
};  // namespace reprojection::database