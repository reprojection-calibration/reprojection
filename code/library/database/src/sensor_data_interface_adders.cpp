#include "database/sensor_data_interface_adders.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

#include "sqlite_wrappers.hpp"
// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "batch_insert.hpp"
#include "serialization.hpp"
#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"

namespace reprojection::database {

void AddCalibrationStep(std::string_view step_name, std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[step_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, step_name); }};

    ExecuteStatement(sql_statements::calibration_steps_insert, binder, database->db);
}

void AddExtractedTargetData(CameraMeasurement const& data, std::string_view sensor_name,
                            std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[&data, sensor_name](sqlite3_stmt* const stmt) {
        auto const& [timestamp_ns, target]{data};

        protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ExtractedTargetProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                                // LCOV_EXCL_LINE
        }

        Sqlite3Tools::Bind(stmt, 1, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 2, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Sqlite3Tools::BindBlob(stmt, 3, std::as_bytes(std::span{buffer}));
    }};

    ExecuteStatement(sql_statements::extracted_target_insert, binder, database->db);
}

void AddPoseData(Frames const& data, std::string_view step_name, std::string_view sensor_name,
                 std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        Sqlite3Tools::Bind(stmt, 1, step_name);
        Sqlite3Tools::Bind(stmt, 2, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Warn cast!

        Sqlite3Tools::Bind(stmt, 4, frame.pose[0]);
        Sqlite3Tools::Bind(stmt, 5, frame.pose[1]);
        Sqlite3Tools::Bind(stmt, 6, frame.pose[2]);
        Sqlite3Tools::Bind(stmt, 7, frame.pose[3]);
        Sqlite3Tools::Bind(stmt, 8, frame.pose[4]);
        Sqlite3Tools::Bind(stmt, 9, frame.pose[5]);
    }};

    BatchInsert(sql_statements::poses_insert, data, binder, database->db);
}

// NOTE(Jack): We suppress the code coverage for the SerializeToString() because I do not know how to malform/change the
// eigen array input to trigger this.
void AddReprojectionError(ReprojectionErrors const& data, std::string_view step_name, std::string_view sensor_name,
                          std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        protobuf_serialization::ArrayX2dProto const serialized{Serialize(frame)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ArrayX2dProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                         // LCOV_EXCL_LINE
        }

        Sqlite3Tools::Bind(stmt, 1, std::string(step_name));
        Sqlite3Tools::Bind(stmt, 2, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer}));
    }};

    BatchInsert(sql_statements::reprojection_error_insert, data, binder, database->db);
}

void AddImuData(ImuMeasurements const& data, std::string_view sensor_name,
                std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        Sqlite3Tools::Bind(stmt, 1, sensor_name);
        Sqlite3Tools::Bind(stmt, 2, static_cast<int64_t>(timestamp_ns));

        Sqlite3Tools::Bind(stmt, 3, frame.angular_velocity[0]);
        Sqlite3Tools::Bind(stmt, 4, frame.angular_velocity[1]);
        Sqlite3Tools::Bind(stmt, 5, frame.angular_velocity[2]);

        Sqlite3Tools::Bind(stmt, 6, frame.linear_acceleration[0]);
        Sqlite3Tools::Bind(stmt, 7, frame.linear_acceleration[1]);
        Sqlite3Tools::Bind(stmt, 8, frame.linear_acceleration[2]);
    }};

    BatchInsert(sql_statements::imu_data_insert, data, binder, database->db);
}

}  // namespace reprojection::database