#include "database/database_write.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <ranges>
#include <string>

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "serialization.hpp"
#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"
#include "toml_converters.hpp"

namespace reprojection::database {

namespace utils {

// TODO(Jack): Combine this one with the one in the database read file?
void BindStepAndSensor(sqlite3_stmt* const stmt, CalibrationStep const step, std::string_view sensor_name) {
    Sqlite3Tools::Bind(stmt, 1, ToString(step));
    Sqlite3Tools::Bind(stmt, 2, sensor_name);
}

// TODO(Jack): This naming is not great but I want this to work for both Eigen Arrays, Vectors and even Matrices that
// have just one column. We use this basic data struct to represent almost everything.
template <typename T>
    requires(T::ColsAtCompileTime == 1)
void BindEigenColumn(sqlite3_stmt* const stmt, int const start_idx, Eigen::DenseBase<T> const& v) {
    for (Eigen::Index i{0}; i < v.size(); ++i) {
        Sqlite3Tools::Bind(stmt, start_idx + static_cast<int>(i), v.derived()(i));
    }
}

}  // namespace utils

void InsertGravity(Array3d const& data, CalibrationStep const step_name, std::string_view sensor_name,
                   SqlitePtr const db) {
    auto const binder{[data, step_name, sensor_name](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        utils::BindEigenColumn<Array3d>(stmt, 3, data);
    }};

    ExecuteStatement(sql_statements::gravity_insert, binder, db);
}

void InsertExtrinsic(Array6d const& data, CalibrationStep const step_name, std::string_view sensor_name,
                     SqlitePtr const db) {
    auto const binder{[data, step_name, sensor_name](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        utils::BindEigenColumn<Array6d>(stmt, 3, data);
    }};

    ExecuteStatement(sql_statements::extrinsics_insert, binder, db);
}

// TODO(Jack): Input arg order consistency.
void InsertStep(CalibrationStep const step_name, std::optional<std::string_view> cache_key,
                std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[step_name, sensor_name, cache_key](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        if (cache_key) {
            Sqlite3Tools::Bind(stmt, 3, *cache_key);
        }
    }};

    ExecuteStatement(sql_statements::calibration_steps_upsert, binder, db);
}

void InsertCameraInfo(CameraInfo const& camera_info, SqlitePtr const db) {
    auto const binder{[camera_info](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, CalibrationStep::CameraInfo, camera_info.sensor_name);
        Sqlite3Tools::Bind(stmt, 3, ToString(camera_info.camera_model));
        Sqlite3Tools::Bind(stmt, 4, camera_info.bounds.v_max);
        Sqlite3Tools::Bind(stmt, 5, camera_info.bounds.u_max);
    }};

    ExecuteStatement(sql_statements::camera_info_insert, binder, db);
}

void InsertTargets(CameraMeasurements const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, target]{data_i};

        protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ExtractedTargetProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                                // LCOV_EXCL_LINE
        }

        utils::BindStepAndSensor(stmt, CalibrationStep::FeatureExtraction, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, timestamp_ns);
        Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::extracted_target_insert, data, binder, db);
}

void InsertIntrinsics(CameraState const& data, CameraModel const camera_model, CalibrationStep const step_name,
                      std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[&data, camera_model, step_name, sensor_name](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, ToString(camera_model));
        Sqlite3Tools::Bind(stmt, 4, ToToml(camera_model, data.intrinsics));
    }};

    ExecuteStatement(sql_statements::camera_intrinsics_insert, binder, db);
}

void InsertImages(EncodedImages const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, buffer]{data_i};

        utils::BindStepAndSensor(stmt, CalibrationStep::ImageLoading, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, timestamp_ns);

        if (buffer.data.empty()) {
            Sqlite3Tools::BindNull(stmt, 4);
        } else {
            Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer.data}));
        }
    }};

    BatchExecuteStatement(sql_statements::image_insert, data, binder, db);
}

void InsertPoses(Frames const& data, CalibrationStep const step_name, std::string_view sensor_name,
                 SqlitePtr const db) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, timestamp_ns);
        utils::BindEigenColumn<Array6d>(stmt, 4, frame.pose);
    }};

    BatchExecuteStatement(sql_statements::poses_insert, data, binder, db);
}

void InsertImuErrors(ImuErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
                     SqlitePtr const db) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, imu_error] = data_i;

        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, timestamp_ns);
        utils::BindEigenColumn<Vector3d>(stmt, 4, imu_error.delta_angular_velocity);
        utils::BindEigenColumn<Vector3d>(stmt, 7, imu_error.delta_linear_acceleration);
    }};

    BatchExecuteStatement(sql_statements::imu_error_insert, data, binder, db);
}

void InsertImuData(ImuMeasurements const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, imu_data] = data_i;

        Sqlite3Tools::Bind(stmt, 1, sensor_name);
        Sqlite3Tools::Bind(stmt, 2, timestamp_ns);
        utils::BindEigenColumn<Vector3d>(stmt, 3, imu_data.angular_velocity);
        utils::BindEigenColumn<Vector3d>(stmt, 6, imu_data.linear_acceleration);
    }};

    BatchExecuteStatement(sql_statements::imu_data_insert, data, binder, db);
}

// NOTE(Jack): We suppress the code coverage for the SerializeToString() because I do not know how to malform/change the
// eigen array input to trigger this.
void InsertReprojectionErrors(ReprojectionErrors const& data, CalibrationStep const step_name,
                              std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        protobuf_serialization::ArrayX2dProto const serialized{Serialize(frame)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error("ArrayX2dProto.SerializeToString() failed for " +  // LCOV_EXCL_LINE
                                     std::string(sensor_name));                         // LCOV_EXCL_LINE
        }

        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, timestamp_ns);
        Sqlite3Tools::BindBlob(stmt, 4, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::reprojection_error_insert, data, binder, db);
}

void InsertTargetInfo(TargetInfo const& target_info, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[target_info, sensor_name](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, CalibrationStep::TargetInfo, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, ToString(target_info.target_type));
        Sqlite3Tools::Bind(stmt, 4, static_cast<int64_t>(target_info.height));
        Sqlite3Tools::Bind(stmt, 5, static_cast<int64_t>(target_info.width));
        Sqlite3Tools::Bind(stmt, 6, target_info.unit_dimension);
        Sqlite3Tools::Bind(stmt, 7, static_cast<int64_t>(target_info.asymmetric));
    }};

    ExecuteStatement(sql_statements::target_info_insert, binder, db);
}

void InsertControlPoints(spline::Matrix2NXd const& data, CalibrationStep const step_name, std::string_view sensor_name,
                         SqlitePtr const db) {
    // NOTE(Jack): This lets use treat the columns of the eigen matrix like a regular type that we can iterate over.
    // This is required to be compatible with BatchExecuteStatement().
    // NOTE(Jack): We cast the column expression to to an Array6d so that we can use BindEigenColumn, if we do not do
    // that we get some crazy errors about trying to template on a column expression. I bet it can be done but I
    // couldn't figure it out quickly.
    auto indexed_control_point_columns{[](auto const& control_points) {
        return std::views::iota(0, static_cast<int>(control_points.cols())) |
               std::views::transform(
                   [&control_points](int const i) { return std::pair{i, Array6d{control_points.col(i)}}; });
    }};

    auto const binder{[step_name, sensor_name](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [i, control_point]{data_i};

        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, static_cast<int64_t>(i));
        utils::BindEigenColumn<Array6d>(stmt, 4, control_point);
    }};

    BatchExecuteStatement(sql_statements::spline_control_points_insert, indexed_control_point_columns(data), binder,
                          db);
}

void InsertTimeHandler(spline::TimeHandler const& data, CalibrationStep const step_name, std::string_view sensor_name,
                       SqlitePtr const db) {
    auto const binder{[data, step_name, sensor_name](sqlite3_stmt* const stmt) {
        utils::BindStepAndSensor(stmt, step_name, sensor_name);
        Sqlite3Tools::Bind(stmt, 3, data.t0_ns_);
        Sqlite3Tools::Bind(stmt, 4, data.delta_t_ns_);
    }};

    ExecuteStatement(sql_statements::spline_time_handler_insert, binder, db);
}

}  // namespace reprojection::database