#include "database/database_write.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "serialization.hpp"
#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"
#include "toml_converters.hpp"

namespace reprojection::database {

void WriteToDb(CameraInfo const& camera_info, SqlitePtr const db) {
    auto const binder{[camera_info](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, "camera_info");
        Sqlite3Tools::Bind(stmt, 2, camera_info.sensor_name);
        Sqlite3Tools::Bind(stmt, 3, ToString(camera_info.camera_model));
        Sqlite3Tools::Bind(stmt, 4, camera_info.bounds.v_max);
        Sqlite3Tools::Bind(stmt, 5, camera_info.bounds.u_max);
    }};

    ExecuteStatement(sql_statements::camera_info_insert, binder, db);
}

// TODO(Jack): Input arg order consistency.
void WriteToDb(CalibrationStep const step_name, std::optional<std::string_view> cache_key, std::string_view sensor_name,
               SqlitePtr const db) {
    auto const binder{[step_name, sensor_name, cache_key](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
        Sqlite3Tools::Bind(stmt, 2, sensor_name);
        if (cache_key) {
            Sqlite3Tools::Bind(stmt, 3, *cache_key);
        }
    }};

    ExecuteStatement(sql_statements::calibration_steps_upsert, binder, db);
}

void WriteToDb(EncodedImages const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, buffer]{data_i};

        Sqlite3Tools::Bind(stmt, 1, ToString(CalibrationStep::ImageLoading));
        Sqlite3Tools::Bind(stmt, 2, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!

        if (buffer.data.empty()) {
            Sqlite3Tools::BindNull(stmt, 4);
        } else {
            Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer.data}));
        }
    }};

    BatchExecuteStatement(sql_statements::image_insert, data, binder, db);
}

void WriteToDb(CameraMeasurements const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, target]{data_i};

        protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ExtractedTargetProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                                // LCOV_EXCL_LINE
        }

        Sqlite3Tools::Bind(stmt, 1, "feature_extraction");
        Sqlite3Tools::Bind(stmt, 2, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::extracted_target_insert, data, binder, db);
}

void WriteToDb(CameraState const& data, CameraModel const camera_model, CalibrationStep const step_name,
               std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[&data, camera_model, step_name, sensor_name](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
        Sqlite3Tools::Bind(stmt, 2, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 3, ToString(camera_model));
        Sqlite3Tools::Bind(stmt, 4, ToToml(camera_model, data.intrinsics));
    }};

    ExecuteStatement(sql_statements::camera_intrinsics_insert, binder, db);
}

void WriteToDb(Frames const& data, CalibrationStep const step_name, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
        Sqlite3Tools::Bind(stmt, 2, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Warn cast!

        Sqlite3Tools::Bind(stmt, 4, frame.pose[0]);
        Sqlite3Tools::Bind(stmt, 5, frame.pose[1]);
        Sqlite3Tools::Bind(stmt, 6, frame.pose[2]);
        Sqlite3Tools::Bind(stmt, 7, frame.pose[3]);
        Sqlite3Tools::Bind(stmt, 8, frame.pose[4]);
        Sqlite3Tools::Bind(stmt, 9, frame.pose[5]);
    }};

    BatchExecuteStatement(sql_statements::poses_insert, data, binder, db);
}

// NOTE(Jack): We suppress the code coverage for the SerializeToString() because I do not know how to malform/change the
// eigen array input to trigger this.
void WriteToDb(ReprojectionErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        protobuf_serialization::ArrayX2dProto const serialized{Serialize(frame)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ArrayX2dProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                         // LCOV_EXCL_LINE
        }

        Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
        Sqlite3Tools::Bind(stmt, 2, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::reprojection_error_insert, data, binder, db);
}

void WriteToDb(ImuMeasurements const& data, std::string_view sensor_name, SqlitePtr const db) {
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

    BatchExecuteStatement(sql_statements::imu_data_insert, data, binder, db);
}

}  // namespace reprojection::database