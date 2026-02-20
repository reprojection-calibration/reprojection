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

void AddCameraPoseData(Frames const& data, std::string_view sensor_name, PoseType const type,
                       std::shared_ptr<CalibrationDatabase> const database) {
    std::string_view const sql{sql_statements::camera_poses_insert};

    AddPoseData(data, sensor_name, type, sql, database);
}

// TODO(Jack): Current hacked interface before we know what the imu data  should really look like!
void AddSplinePoseData(SplinePoses const& data, std::string_view sensor_name, PoseType const type,
                       std::shared_ptr<CalibrationDatabase> const database) {
    std::string_view const sql{sql_statements::spline_poses_insert};

    // TODO(Jack): For now we will simply convert SplinePoses to CameraCalibrationData. Once we know better the
    //  requirements or have a better design concept we can remove this conversion code. This is a hack!
    Frames hack_data;
    for (auto const& [timestamp_ns, pose_i] : data) {
        hack_data[timestamp_ns].pose = pose_i;
    }

    AddPoseData(hack_data, sensor_name, type, sql, database);
}

// NOTE(Jack): We suppress the code coverage for SqliteErrorCode::FailedBinding because the only way I know how to
// trigger that is via a malformed sql statement, but that is hardcoded into this function (i.e.
// sql_statements::camera_poses_insert) abd cannot and should not be changed!
void AddPoseData(Frames const& data, std::string_view sensor_name, PoseType const type, std::string_view sql,
                 std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, frame_i] : data) {
        // TODO(Jack): Make SqlStatement take a string view.
        SqlStatement const statement{database->db, sql.data()};

        try {
            Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Warn cast!
            Sqlite3Tools::Bind(statement.stmt, 2, sensor_name);
            Sqlite3Tools::Bind(statement.stmt, 3, ToString(type));
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
void AddReprojectionError(std::map<uint64_t, ArrayX2d> const& data, std::string_view sensor_name, PoseType const type,
                          std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& [timestamp_ns, error_i] : data) {
        protobuf_serialization::ArrayX2dProto const serialized{Serialize(error_i)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error(
                "AddReprojectionError() protobuf SerializeToString() failed for sensor: " +       // LCOV_EXCL_LINE
                std::string(sensor_name) + " at timestamp_ns: " + std::to_string(timestamp_ns));  // LCOV_EXCL_LINE
        }

        SqliteResult const result{Sqlite3Tools::AddTimeNameTypeBlob(sql_statements::reprojection_error_insert,
                                                                    timestamp_ns, type, sensor_name, buffer.c_str(),
                                                                    std::size(buffer), database->db)};

        if (std::holds_alternative<SqliteErrorCode>(result)) {
            throw std::runtime_error(ErrorMessage("AddReprojectionError()", sensor_name, timestamp_ns,
                                                  std::get<SqliteErrorCode>(result),
                                                  std::string(sqlite3_errmsg(database->db))));
        }
    }
}

// NOTE(Jack): See note above AddReprojectionError about suppressing the SerializeToString throw.
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

    SqliteResult const result{Sqlite3Tools::AddTimeNameBlob(sql_statements::extracted_target_insert, timestamp_ns,
                                                            sensor_name, buffer.c_str(), std::size(buffer),
                                                            database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error(ErrorMessage("AddReprojectionError()", sensor_name, timestamp_ns,
                                              std::get<SqliteErrorCode>(result),
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

// NOTE(Jack): The core sql handling logic here is very similar to the ImageStreamer class, but there are enough
// differences that we cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
// NOTE(Jack): See notes above to understand why we suppress code coverage.
CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name) {
    SqlStatement const statement{database->db, sql_statements::extracted_targets_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, std::string(sensor_name).c_str());
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

// TODO(Jack): Remove bool return to make consistent with other methods
[[nodiscard]] bool AddImuData(ImuMeasurement const& data, std::string_view sensor_name,
                              std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::imu_data_insert};

    auto const& [timestamp_ns, imu_data]{data};
    try {
        Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Warn cast!
        Sqlite3Tools::Bind(statement.stmt, 2, sensor_name);
        Sqlite3Tools::Bind(statement.stmt, 3, imu_data.angular_velocity[0]);
        Sqlite3Tools::Bind(statement.stmt, 4, imu_data.angular_velocity[1]);
        Sqlite3Tools::Bind(statement.stmt, 5, imu_data.angular_velocity[2]);
        Sqlite3Tools::Bind(statement.stmt, 6, imu_data.linear_acceleration[0]);
        Sqlite3Tools::Bind(statement.stmt, 7, imu_data.linear_acceleration[1]);
        Sqlite3Tools::Bind(statement.stmt, 8, imu_data.linear_acceleration[2]);
    } catch (std::runtime_error const& e) {                      // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(ErrorMessage(  // LCOV_EXCL_LINE
            "AddImuData()", sensor_name,                         // LCOV_EXCL_LINE
            timestamp_ns, SqliteErrorCode::FailedBinding,        // LCOV_EXCL_LINE
            std::string(sqlite3_errmsg(database->db)))));        // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        throw std::runtime_error(ErrorMessage("AddImuData()", sensor_name, timestamp_ns, SqliteErrorCode::FailedStep,
                                              std::string(sqlite3_errmsg(database->db))));
    }

    return true;
}

// TODO(Jack): Update to void and throw interface and consistent error messages!
// TODO(Jack): Remove use of set!
ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name) {
    SqlStatement const statement{database->db, sql_statements::imu_data_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, std::string(sensor_name).c_str());
    } catch (std::runtime_error const& e) {                             // LCOV_EXCL_LINE
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