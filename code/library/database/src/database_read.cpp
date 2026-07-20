#include "database/database_read.hpp"

#include <sqlite3.h>

// cppcheck-suppress missingInclude
#include "../include/database/serialization.hpp"
#include "generated/sql.hpp"

#include "query_runner.hpp"
#include "sqlite3_helpers.hpp"
#include "toml_converters.hpp"

namespace reprojection::database {

namespace utils {

// NOTE(Jack): we have this little utils namespace because the following two segments of code showed up constantly and
// these two functions helper us eliminate a lot of repetition.

template <int N>
Eigen::Array<double, N, 1> ColumnArray(sqlite3_stmt* const stmt, int const start_idx) {
    Eigen::Array<double, N, 1> result;
    for (int i{0}; i < N; ++i) {
        result(i) = sqlite3_column_double(stmt, start_idx + i);
    }
    return result;
}

// TODO(Jack): This is mostly duplicated in the write utils (except that it's not a lambda), can we combine these?
auto BindStepAndSensor(CalibrationStep const step_name, std::string_view sensor_name) {
    return [step_name, sensor_name](sqlite3_stmt* stmt) {
        Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
        Sqlite3Tools::Bind(stmt, 2, sensor_name);
    };
}

}  // namespace utils

std::optional<std::string> ReadCacheKey(SqlitePtr const db, std::string_view sensor_name,
                                        CalibrationStep const step_name) {
    std::optional<std::string> cache_key;

    ExecuteQuery(db, sql_statements::calibration_steps_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&cache_key](sqlite3_stmt* const stmt) {
                     // TODO(Jack): Is the best way to express reading a value which might be null from a table? I do
                     // not see the problem with this implementation, but maybe sqlite has some pattern I don't know
                     // about.
                     uchar const* const value{sqlite3_column_text(stmt, 0)};
                     if (value) {
                         cache_key = std::string(reinterpret_cast<char const*>(value));
                     }
                 });

    return cache_key;
}  // LCOV_EXCL_LINE

// TODO(Jack): Write helper to convert a text column to a string, we copy paste this logic in several places, same for
//  the timestamp int field.
// NOTE(Jack): We hardcode the minimum image height/width values to zero here, I think that is not a problem, but lets
// not forget that we do it here.
std::optional<CameraInfo> ReadCameraInfo(SqlitePtr const db, std::string_view sensor_name) {
    std::optional<CameraInfo> camera_info;

    ExecuteQuery(
        db, sql_statements::camera_info_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&camera_info](sqlite3_stmt* const stmt) {
            CameraInfo result;
            result.camera_model = ToCameraModel(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0)));
            result.bounds.v_max = sqlite3_column_int(stmt, 1);
            result.bounds.v_min = 0;
            result.bounds.u_max = sqlite3_column_int(stmt, 2);
            result.bounds.u_min = 0;

            camera_info = result;
        });

    if (camera_info.has_value()) {
        camera_info->sensor_name = sensor_name;
    }

    return camera_info;
}  // LCOV_EXCL_LINE

std::optional<ArrayXd> ReadIntrinsics(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                                      CameraModel const camera_model) {
    std::optional<ArrayXd> intrinsics;

    ExecuteQuery(
        db, sql_statements::camera_intrinsics_select,
        [step_name, sensor_name, camera_model](sqlite3_stmt* const stmt) {  // LCOV_EXCL_LINE
            Sqlite3Tools::Bind(stmt, 1, ToString(step_name));
            Sqlite3Tools::Bind(stmt, 2, sensor_name);
            Sqlite3Tools::Bind(stmt, 3, ToString(camera_model));
        },
        [&intrinsics, camera_model](sqlite3_stmt* const stmt) {
            intrinsics =
                FromToml(camera_model, std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0))));
        });

    return intrinsics;
}  // LCOV_EXCL_LINE

EncodedImages ReadImages(SqlitePtr const db, std::string_view sensor_name) {
    EncodedImages images;

    ExecuteQuery(
        db, sql_statements::images_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&images](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{Sqlite3Tools::SqliteBlob(stmt, 1)};
            std::span<uchar const> blob_span{reinterpret_cast<uchar const*>(blob.data()), blob.size()};
            std::vector<uchar> buffer(std::cbegin(blob_span), std::cend(blob_span));

            // TODO(Jack): Should we represent empty images with std::optional? Currently this will load all images, and
            // if the image is a null value it will just be a buffer with length zero. This is not exactly a problem but
            // it might be more consistent to explicitly label these empty images with std::optional. Otherwise as is
            // all downstream users will need to manually check if the buffer is empty before attempting to decode it to
            // make sure there is actually an image in there.
            images.insert({timestamp_ns, ImageBuffer{buffer}});
        });

    return images;
}  // LCOV_EXCL_LINE

// NOTE(Jack): See notes above to understand why we suppress code coverage.
CameraMeasurements ReadTargets(SqlitePtr const db, std::string_view sensor_name) {
    CameraMeasurements targets;

    ExecuteQuery(
        db, sql_statements::extracted_targets_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&targets, sensor_name](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{Sqlite3Tools::SqliteBlob(stmt, 1)};
            protobuf_serialization::ExtractedTargetProto serialized;
            serialized.ParseFromArray(std::data(blob), static_cast<int>(std::size(blob)));

            auto const deserialized{Deserialize(serialized)};
            if (not deserialized) {
                throw std::runtime_error("Deserialize(ExtractedTargetProto) failed for " +  // LCOV_EXCL_LINE
                                         std::string(sensor_name));                         // LCOV_EXCL_LINE
            }

            targets.insert({timestamp_ns, deserialized.value()});
        });

    return targets;
}  // LCOV_EXCL_LINE

std::optional<Extrinsic> ReadExtrinsics(SqlitePtr const db, std::string_view sensor_name,
                                        CalibrationStep const step_name) {
    std::optional<Extrinsic> extrinsic;

    ExecuteQuery(db, sql_statements::extrinsics_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&extrinsic](sqlite3_stmt* const stmt) {
                     std::string const frame_a{reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0))};
                     std::string const frame_b{reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1))};
                     Array6d const tf_a_b = utils::ColumnArray<6>(stmt, 2);

                     extrinsic = Extrinsic{frame_a, frame_b, tf_a_b};
                 });

    return extrinsic;
}  // LCOV_EXCL_LINE

std::optional<Array3d> ReadGravity(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name) {
    std::optional<Array3d> gravity;

    ExecuteQuery(db, sql_statements::gravity_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&gravity](sqlite3_stmt* const stmt) { gravity = utils::ColumnArray<3>(stmt, 0); });

    return gravity;
}

ImuMeasurements ReadImuData(SqlitePtr const db, std::string_view sensor_name) {
    ImuMeasurements imu_data;

    ExecuteQuery(
        db, sql_statements::imu_data_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&imu_data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
            Array6d const loaded{utils::ColumnArray<6>(stmt, 1)};

            imu_data.insert(ImuMeasurement{timestamp_ns, {loaded.topRows<3>(), loaded.bottomRows<3>()}});
        });

    return imu_data;
}  // LCOV_EXCL_LINE

// TODO(Jack): This looks really similar to the ImuMeasurement version, and in general we are starting to see a lot of
// repetition here, lets think about if there is anything we can to simplify here.
ImuErrors ReadImuErrors(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name) {
    ImuErrors imu_errors;

    ExecuteQuery(db, sql_statements::imu_errors_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&imu_errors](sqlite3_stmt* const stmt) {
                     uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
                     Array6d const loaded{utils::ColumnArray<6>(stmt, 1)};

                     imu_errors.insert(ImuError{timestamp_ns, {loaded.topRows<3>(), loaded.bottomRows<3>()}});
                 });

    return imu_errors;
}  // LCOV_EXCL_LINE

Frames ReadPoses(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name) {
    Frames poses;

    ExecuteQuery(db, sql_statements::poses_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&poses](sqlite3_stmt* const stmt) {
                     uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
                     Array6d const loaded{utils::ColumnArray<6>(stmt, 1)};

                     poses.insert(Frame{timestamp_ns, loaded});
                 });

    return poses;
}  // LCOV_EXCL_LINE

std::optional<TargetInfo> ReadTargetInfo(SqlitePtr const db, std::string_view sensor_name) {
    std::optional<TargetInfo> target_info;

    ExecuteQuery(
        db, sql_statements::target_info_select,
        [sensor_name](sqlite3_stmt* const stmt) { Sqlite3Tools::Bind(stmt, 1, sensor_name); },
        [&target_info](sqlite3_stmt* const stmt) {
            TargetInfo result;
            result.target_type = ToTargetType(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0)));
            result.height = sqlite3_column_int(stmt, 1);
            result.width = sqlite3_column_int(stmt, 2);
            result.unit_dimension = sqlite3_column_double(stmt, 3);
            result.asymmetric = static_cast<bool>(sqlite3_column_int(stmt, 4));

            target_info = result;
        });

    return target_info;
}

spline::Matrix2NXd ReadControlPoints(SqlitePtr const db, std::string_view sensor_name,
                                     CalibrationStep const step_name) {
    // First we need to recover how many control points there are so we can size the control point matrix properly.
    int64_t num_control_points{-1};
    ExecuteQuery(db, sql_statements::spline_control_points_count, utils::BindStepAndSensor(step_name, sensor_name),
                 [&num_control_points](sqlite3_stmt* const stmt) {
                     num_control_points = static_cast<uint64_t>(sqlite3_column_int64(stmt, 0));
                 });

    // Now get the actual control points and put them into the pre-sized eigen matrix :)
    spline::Matrix2NXd control_points(6, num_control_points);
    ExecuteQuery(db, sql_statements::spline_control_points_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&control_points](sqlite3_stmt* const stmt) {
                     int64_t const id{sqlite3_column_int64(stmt, 0)};
                     control_points.col(id) = utils::ColumnArray<6>(stmt, 1);
                 });

    return control_points;
}  // LCOV_EXCL_LINE

std::optional<spline::TimeHandler> ReadTimeHandler(SqlitePtr const db, std::string_view sensor_name,
                                                   CalibrationStep const step_name) {
    std::optional<spline::TimeHandler> time_handler;

    ExecuteQuery(db, sql_statements::spline_time_handler_select, utils::BindStepAndSensor(step_name, sensor_name),
                 [&time_handler](sqlite3_stmt* const stmt) {
                     uint64_t const t0_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
                     uint64_t const delta_t_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 1))};

                     time_handler = spline::TimeHandler{t0_ns, delta_t_ns};
                 });

    return time_handler;
}

};  // namespace reprojection::database