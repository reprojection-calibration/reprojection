#include "database/sensor_data_interface.hpp"

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

// NOTE(Jack): We supress the code coverage for SqliteErrorCode::FailedBinding because the only way I know how to
// trigger that is via a malformed sql statement, but that is hardcoded into this function (i.e.
// sql_statements::camera_poses_insert) abd cannot and should not be changed!
void AddPoseData(CameraCalibrationData const& data, PoseType const type,
                 std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        SqlStatement const statement{database->db, sql_statements::camera_poses_insert};

        Vector6d pose;
        if (type == PoseType::Initial) {
            // TODO IMPLEMENT A REAL STRATEGY HERE! Should we actually have two pose tables so we can establish a
            // foreign key constraint between initialized and optimized?
            if (frame_i.initial_pose.has_value()) {
                pose = frame_i.initial_pose.value();
            } else {
                continue;
            }
        } else if (type == PoseType::Optimized) {
            pose = frame_i.optimized_pose;
        } else {
            throw std::runtime_error(
                "AddPoseData() invalid PoseType selected, this is a library implementation error!");  // LCOV_EXCL_LINE
        }

        try {
            Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Warn cast!
            Sqlite3Tools::Bind(statement.stmt, 2, data.sensor.sensor_name);
            Sqlite3Tools::Bind(statement.stmt, 3, ToString(type));
            Sqlite3Tools::Bind(statement.stmt, 4, pose[0]);
            Sqlite3Tools::Bind(statement.stmt, 5, pose[1]);
            Sqlite3Tools::Bind(statement.stmt, 6, pose[2]);
            Sqlite3Tools::Bind(statement.stmt, 7, pose[3]);
            Sqlite3Tools::Bind(statement.stmt, 8, pose[4]);
            Sqlite3Tools::Bind(statement.stmt, 9, pose[5]);
        } catch (std::runtime_error const& e) {                                       // LCOV_EXCL_LINE
            std::throw_with_nested(std::runtime_error(                                // LCOV_EXCL_LINE
                ErrorMessage("AddPoseData()", data.sensor.sensor_name, timestamp_ns,  // LCOV_EXCL_LINE
                             SqliteErrorCode::FailedBinding,                          // LCOV_EXCL_LINE
                             std::string(sqlite3_errmsg(database->db)))));            // LCOV_EXCL_LINE
        }  // LCOV_EXCL_LINE

        if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
            throw std::runtime_error(ErrorMessage("AddPoseData()", data.sensor.sensor_name, timestamp_ns,
                                                  SqliteErrorCode::FailedStep,
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

// NOTE(Jack): We supress the code coverage for the SerializeToString() because I do not know how to malform/change the
// eigen array input to trigger this.
void AddReprojectionError(CameraCalibrationData const& data, PoseType const type,
                          std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        // TODO(Jack): Can we make this a reference somehow? There is no good reason to make a copy here.
        ArrayX2d reprojection_error;
        if (type == PoseType::Initial) {
            reprojection_error = frame_i.initial_reprojection_error;
        } else if (type == PoseType::Optimized) {
            reprojection_error = frame_i.optimized_reprojection_error;
        } else {
            throw std::runtime_error(
                "AddReprojectionError() invalid PoseType selected, this is a library implementation error!");  // LCOV_EXCL_LINE
        }

        protobuf_serialization::ArrayX2dProto const serialized{Serialize(reprojection_error)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error(
                "AddReprojectionError() protobuf SerializeToString() failed for sensor: " +      // LCOV_EXCL_LINE
                data.sensor.sensor_name + " at timestamp_ns: " + std::to_string(timestamp_ns));  // LCOV_EXCL_LINE
        }

        SqliteResult const result{Sqlite3Tools::AddTimeNameTypeBlob(sql_statements::reprojection_error_insert,
                                                                    timestamp_ns, type, data.sensor.sensor_name,
                                                                    buffer.c_str(), std::size(buffer), database->db)};

        if (std::holds_alternative<SqliteErrorCode>(result)) {
            throw std::runtime_error(ErrorMessage("AddReprojectionError()", data.sensor.sensor_name, timestamp_ns,
                                                  std::get<SqliteErrorCode>(result),
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

// NOTE(Jack): See note above AddReprojectionError about suppressing the SerializeToString throw.
void AddExtractedTargetData(ExtractedTargetStamped const& data, std::shared_ptr<CalibrationDatabase> const database) {
    protobuf_serialization::ExtractedTargetProto const serialized{Serialize(data.target)};
    std::string buffer;
    if (not serialized.SerializeToString(&buffer)) {
        throw std::runtime_error(
            "AddExtractedTargetData() protobuf SerializeToString() failed for sensor: " +  // LCOV_EXCL_LINE
            data.header.sensor_name +                                                      // LCOV_EXCL_LINE
            " at timestamp_ns: " + std::to_string(data.header.timestamp_ns));              // LCOV_EXCL_LINE
    }

    SqliteResult const result{Sqlite3Tools::AddTimeNameBlob(sql_statements::extracted_target_insert,
                                                            data.header.timestamp_ns, data.header.sensor_name,
                                                            buffer.c_str(), std::size(buffer), database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error(ErrorMessage("AddReprojectionError()", data.header.sensor_name,
                                              data.header.timestamp_ns, std::get<SqliteErrorCode>(result),
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

// NOTE(Jack): The core sql handling logic here is very similar to the ImageStreamer class, but there are enough
// differences that we cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
// NOTE(Jack): See notes above to understand why we suppress code coverage.
void GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database, CameraCalibrationData& data) {
    SqlStatement const statement{database->db, sql_statements::extracted_targets_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, data.sensor.sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                       // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(                                // LCOV_EXCL_LINE
            ErrorMessage("GetExtractedTargetData()", data.sensor.sensor_name, 0,  // LCOV_EXCL_LINE
                         SqliteErrorCode::FailedBinding,                          // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database->db)))));            // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            throw std::runtime_error(ErrorMessage(                                         // LCOV_EXCL_LINE
                "GetExtractedTargetData()", data.sensor.sensor_name, 0,                    // LCOV_EXCL_LINE
                SqliteErrorCode::FailedStep, std::string(sqlite3_errmsg(database->db))));  // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be more defensive here and first check that column text is not returning a nullptr or
        // other bad output? Also happens like this in the image streamer and possibly other places.
        uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(statement.stmt, 0)))};

        uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(statement.stmt, 1))};
        int const blob_size{sqlite3_column_bytes(statement.stmt, 1)};
        if (not blob or blob_size <= 0) {
            throw std::runtime_error("GetExtractedTargetData() blob reading failed for sensor: " +  // LCOV_EXCL_LINE
                                     data.sensor.sensor_name +                                      // LCOV_EXCL_LINE
                                     " at timestamp_ns: " + std::to_string(timestamp_ns));          // LCOV_EXCL_LINE
        }

        std::vector<uchar> const buffer(blob, blob + blob_size);
        protobuf_serialization::ExtractedTargetProto serialized;
        serialized.ParseFromArray(buffer.data(), buffer.size());

        auto const deserialized{Deserialize(serialized)};
        if (not deserialized.has_value()) {
            throw std::runtime_error("GetExtractedTargetData() Deserialize() failed for sensor: " +  // LCOV_EXCL_LINE
                                     data.sensor.sensor_name +                                       // LCOV_EXCL_LINE
                                     " at timestamp_ns: " + std::to_string(timestamp_ns));           // LCOV_EXCL_LINE
        }

        data.frames[timestamp_ns].extracted_target = deserialized.value();
    }
}

// TODO(Jack): Update to void and throw interface and consistent error messages!
[[nodiscard]] bool AddImuData(ImuStamped const& data, std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::imu_data_insert};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(data.header.timestamp_ns));  // Warn cast!
        Sqlite3Tools::Bind(statement.stmt, 2, data.header.sensor_name);
        Sqlite3Tools::Bind(statement.stmt, 3, data.angular_velocity[0]);
        Sqlite3Tools::Bind(statement.stmt, 4, data.angular_velocity[1]);
        Sqlite3Tools::Bind(statement.stmt, 5, data.angular_velocity[2]);
        Sqlite3Tools::Bind(statement.stmt, 6, data.linear_acceleration[0]);
        Sqlite3Tools::Bind(statement.stmt, 7, data.linear_acceleration[1]);
        Sqlite3Tools::Bind(statement.stmt, 8, data.linear_acceleration[2]);
    } catch (std::runtime_error const& e) {                            // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(ErrorMessage(        // LCOV_EXCL_LINE
            "AddImuData()", data.header.sensor_name,                   // LCOV_EXCL_LINE
            data.header.timestamp_ns, SqliteErrorCode::FailedBinding,  // LCOV_EXCL_LINE
            std::string(sqlite3_errmsg(database->db)))));              // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        throw std::runtime_error(ErrorMessage("AddImuData()", data.header.sensor_name, data.header.timestamp_ns,
                                              SqliteErrorCode::FailedStep, std::string(sqlite3_errmsg(database->db))));
    }

    return true;
}

// TODO(Jack): Update to void and throw interface and consistent error messages!
std::optional<std::set<ImuStamped>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                               std::string const& sensor_name) {
    SqlStatement const statement{database->db, sql_statements::imu_data_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                                        // LCOV_EXCL_LINE
        std::cerr << ErrorMessage("GetImuData()", sensor_name, 0, SqliteErrorCode::FailedBinding,  // LCOV_EXCL_LINE
                                  std::string(sqlite3_errmsg(database->db)))                       // LCOV_EXCL_LINE
                  << "\n";                                                                         // LCOV_EXCL_LINE
        return std::nullopt;                                                                       // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    std::set<ImuStamped> data;
    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            std::cerr << ErrorMessage("GetImuData()", sensor_name, 0, SqliteErrorCode::FailedStep,  // LCOV_EXCL_LINE
                                      std::string(sqlite3_errmsg(database->db)))                    // LCOV_EXCL_LINE
                      << "\n";                                                                      // LCOV_EXCL_LINE
            return std::nullopt;                                                                    // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be doing any error checking here while reading the columns?
        uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(statement.stmt, 0))};  // Warn cast!
        double const omega_x{sqlite3_column_double(statement.stmt, 1)};
        double const omega_y{sqlite3_column_double(statement.stmt, 2)};
        double const omega_z{sqlite3_column_double(statement.stmt, 3)};
        double const ax{sqlite3_column_double(statement.stmt, 4)};
        double const ay{sqlite3_column_double(statement.stmt, 5)};
        double const az{sqlite3_column_double(statement.stmt, 6)};

        data.insert(ImuStamped{{timestamp_ns, sensor_name}, {omega_x, omega_y, omega_z}, {ax, ay, az}});
    }

    return data;
}
};  // namespace reprojection::database