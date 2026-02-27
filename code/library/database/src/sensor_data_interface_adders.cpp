#include "database/sensor_data_interface_adders.hpp"

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

void AddCalibrationStep(std::string_view step_name, std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::calibration_steps_insert};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, step_name);
    } catch (std::runtime_error const& e) {
        // TODO(Jack): Clearly the ErrorMessage() has outlived its useful scope because we have to pass in dummy values
        //  here and below.
        std::throw_with_nested(std::runtime_error(                      // LCOV_EXCL_LINE
            ErrorMessage("AddCalibrationStep()", "N/A", 0,              // LCOV_EXCL_LINE
                         SqliteErrorCode::FailedBinding,                // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database->db)))));  // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        throw std::runtime_error(ErrorMessage("AddCalibrationStep()", "N/A", 0, SqliteErrorCode::FailedStep,
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

// NOTE(Jack): See note in AddReprojectionError about suppressing the SerializeToString throw.
void AddExtractedTargetData(CameraMeasurement const& data, std::string_view sensor_name,
                            std::shared_ptr<CalibrationDatabase> const database) {
    auto const& [timestamp_ns, target]{data};

    protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
    std::string buffer;
    if (not serialized.SerializeToString(&buffer)) {
        throw std::runtime_error(
            "AddExtractedTargetData() protobuf SerializeToString() failed for sensor: " +  // LCOV_EXCL_LINE
            std::string(sensor_name) +                                                     // LCOV_EXCL_LINE
            " at timestamp_ns: " + std::to_string(timestamp_ns));                          // LCOV_EXCL_LINE
    }

    SqliteResult const result{Sqlite3Tools::AddTimeNameBlob(sql_statements::extracted_target_insert, sensor_name,
                                                            timestamp_ns, buffer.c_str(), std::size(buffer),
                                                            database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error(ErrorMessage("AddReprojectionError()", sensor_name, timestamp_ns,
                                              std::get<SqliteErrorCode>(result),
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

// NOTE(Jack): We suppress the code coverage for SqliteErrorCode::FailedBinding because the only way I know how to
// trigger that is via a malformed sql statement, but that is hardcoded into this function (i.e.
// sql_statements::camera_poses_insert) abd cannot and should not be changed!
void AddPoseData(Frames const& data, std::string_view step_name, std::string_view sensor_name,
                 std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, frame_i] : data) {
        SqlStatement const statement{database->db, sql_statements::poses_insert};

        // TODO(Jack): Refactor everywhere order to be step_name, sensor_name, timestamp
        try {
            Sqlite3Tools::Bind(statement.stmt, 1, step_name);
            Sqlite3Tools::Bind(statement.stmt, 2, sensor_name);
            Sqlite3Tools::Bind(statement.stmt, 3, static_cast<int64_t>(timestamp_ns));  // Warn cast!
            Sqlite3Tools::Bind(statement.stmt, 4, frame_i.pose[0]);
            Sqlite3Tools::Bind(statement.stmt, 5, frame_i.pose[1]);
            Sqlite3Tools::Bind(statement.stmt, 6, frame_i.pose[2]);
            Sqlite3Tools::Bind(statement.stmt, 7, frame_i.pose[3]);
            Sqlite3Tools::Bind(statement.stmt, 8, frame_i.pose[4]);
            Sqlite3Tools::Bind(statement.stmt, 9, frame_i.pose[5]);
        } catch (std::runtime_error const& e) {                             // LCOV_EXCL_LINE
            std::throw_with_nested(std::runtime_error(                      // LCOV_EXCL_LINE
                ErrorMessage("AddPoseData()", sensor_name, timestamp_ns,    // LCOV_EXCL_LINE
                             SqliteErrorCode::FailedBinding,                // LCOV_EXCL_LINE
                             std::string(sqlite3_errmsg(database->db)))));  // LCOV_EXCL_LINE
        }  // LCOV_EXCL_LINE

        if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
            throw std::runtime_error(ErrorMessage("AddPoseData()", sensor_name, timestamp_ns,
                                                  SqliteErrorCode::FailedStep,
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

// NOTE(Jack): We suppress the code coverage for the SerializeToString() because I do not know how to malform/change the
// eigen array input to trigger this.
void AddReprojectionError(ReprojectionErrors const& data, std::string_view step_name, std::string_view sensor_name,
                          std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, error_i] : data) {
        protobuf_serialization::ArrayX2dProto const serialized{Serialize(error_i)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            // TODO(Jack): Should also add step_name?
            throw std::runtime_error(
                "AddReprojectionError() protobuf SerializeToString() failed for sensor: " +       // LCOV_EXCL_LINE
                std::string(sensor_name) + " at timestamp_ns: " + std::to_string(timestamp_ns));  // LCOV_EXCL_LINE
        }

        SqliteResult const result{Sqlite3Tools::AddStepTimeNameBlob(sql_statements::reprojection_error_insert,
                                                                    step_name, sensor_name, timestamp_ns,
                                                                    buffer.c_str(), std::size(buffer), database->db)};

        if (std::holds_alternative<SqliteErrorCode>(result)) {
            // TODO(Jack): Should also add step_name?
            throw std::runtime_error(ErrorMessage("AddReprojectionError()", sensor_name, timestamp_ns,
                                                  std::get<SqliteErrorCode>(result),
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

void AddImuData(ImuMeasurements const& data, std::string_view sensor_name,
                std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, frame_i] : data) {
        SqlStatement const statement{database->db, sql_statements::imu_data_insert};

        try {
            Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Warn cast!
            Sqlite3Tools::Bind(statement.stmt, 2, sensor_name);
            Sqlite3Tools::Bind(statement.stmt, 3, frame_i.angular_velocity[0]);
            Sqlite3Tools::Bind(statement.stmt, 4, frame_i.angular_velocity[1]);
            Sqlite3Tools::Bind(statement.stmt, 5, frame_i.angular_velocity[2]);
            Sqlite3Tools::Bind(statement.stmt, 6, frame_i.linear_acceleration[0]);
            Sqlite3Tools::Bind(statement.stmt, 7, frame_i.linear_acceleration[1]);
            Sqlite3Tools::Bind(statement.stmt, 8, frame_i.linear_acceleration[2]);
        } catch (std::runtime_error const& e) {                      // LCOV_EXCL_LINE
            std::throw_with_nested(std::runtime_error(ErrorMessage(  // LCOV_EXCL_LINE
                "AddImuData()", sensor_name,                         // LCOV_EXCL_LINE
                timestamp_ns, SqliteErrorCode::FailedBinding,        // LCOV_EXCL_LINE
                std::string(sqlite3_errmsg(database->db)))));        // LCOV_EXCL_LINE
        }  // LCOV_EXCL_LINE

        if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
            throw std::runtime_error(ErrorMessage("AddImuData()", sensor_name, timestamp_ns,
                                                  SqliteErrorCode::FailedStep,
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

}  // namespace reprojection::database