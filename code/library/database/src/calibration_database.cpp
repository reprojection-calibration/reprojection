#include "database/calibration_database.hpp"

#include <format>
#include <ranges>

#include "database/sqlite_exception.hpp"
// cppcheck-suppress missingInclude
#include "generated/sql.hpp"
#include "hashing/serialize.hpp"

#include "database_semantics.hpp"
#include "serialization.hpp"
#include "sqlite_helpers.hpp"
#include "toml_converters.hpp"

namespace reprojection::database {

SqlitePtr OpenCalibrationDatabase(std::filesystem::path const& db_path, bool const create, bool const read_only) {
    if (create and read_only) {
        throw std::runtime_error(
            "You requested to open a database object with both options 'create' and 'read_only' true. This is "
            "an invalid combination as creating a database requires writing to it!");
    }

    sqlite3* db{nullptr};
    int code;
    if (create) {
        code = sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr);
    } else if (read_only) {
        code = sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr);
    } else {
        code = sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READWRITE, nullptr);
    }

    if (code != SQLITE_OK) {
        // TODO(Jack): Is it valid here to try to get an error message here from an improperly opened db pointer?
        // TODO(Jack): Do we need to close the database here?
        throw SqliteException(db);  // LCOV_EXCL_LINE
    }

    if (not read_only) {
        ExecuteStatement(sql_statements::assets_table, db);
        ExecuteStatement(sql_statements::camera_info_table, db);
        ExecuteStatement(sql_statements::camera_poses_table, db);
        ExecuteStatement(sql_statements::control_points_table, db);
        ExecuteStatement(sql_statements::extracted_targets_table, db);
        ExecuteStatement(sql_statements::extrinsics_table, db);
        ExecuteStatement(sql_statements::gravity_table, db);
        ExecuteStatement(sql_statements::images_table, db);
        ExecuteStatement(sql_statements::imu_data_table, db);
        ExecuteStatement(sql_statements::intrinsics_table, db);
        ExecuteStatement(sql_statements::reprojection_errors_table, db);
        ExecuteStatement(sql_statements::spline_info_table, db);
        ExecuteStatement(sql_statements::steps_table, db);
        ExecuteStatement(sql_statements::target_info_table, db);
        ExecuteStatement(sql_statements::workflow_assets_table, db);
        ExecuteStatement(sql_statements::workflow_steps_table, db);
        ExecuteStatement(sql_statements::workflows_table, db);

        // This trigger enforces that when a step becomes unreferenced (i.e. it does not belong to any workflow) it gets
        // deleted. This should keep the database clean.
        ExecuteStatement(sql_statements::steps_delete_trigger, db);
    }

    // NOTE(Jack): We use the foreign key constraint between some tables to enforce data consistency. For
    // example a row in initial_camera_poses can only possibly exist if there is a corresponding entry in
    // extracted_targets. And that row in the extracted_targets table can only possibly exist if there is a
    // corresponding entry in the images table.
    //
    // That being said sqlite has the foreign key option off by default (https://sqlite.org/foreignkeys.html) so
    // we need to manually turn it on here.
    ExecuteStatement("PRAGMA foreign_keys = ON;", db);

    // WARN(Jack): This lambda here is our way of ensuring (at least I hope so), the proper closure/destruction of the
    // db. Note that every place that we create a SqlitePtr we need to pass this lambda which is a little hacky. But
    // hopefully this function is the only function we ever use to open a calibration database and therefore it won't be
    // a problem.
    return SqlitePtr{db, [](sqlite3* const db) { sqlite3_close_v2(db); }};
}

AssetId GetOrCreateAsset(sqlite3* const db, AssetType const type, size_t const index, Name const& name) {
    auto const result{ReadAssetId(db, type, index)};
    if (result and result->second != name) {
        throw std::runtime_error(
            std::format("Asset of type '{}', index '{}' and name '{}' already exists - cannot change name '{}'.",
                        ToString(type), index, result->second.value, name.value));
    } else if (result) {
        return result->first;
    }

    return InsertAsset(db, type, index, name);
}

void DeleteUnusedAssets(sqlite3* const db) { ExecuteStatement(sql_statements::assets_delete, db); }

WorkflowId GetOrCreateWorkflow(sqlite3* const db, WorkflowType const type, std::vector<AssetId> const& assets) {
    // NOTE(Jack): We do not hash the signature so that way it remains humand readable in the database. It is a small
    // and important piece of information so this just makes sense.
    // TODO(Jack): Is there a better way of uniquely identifying the workflow than creating this string of the assets
    // here? I am not sure how this will scale to multisensor cases but that is still future music. The main problem I
    // have is that this leaves information/constraints on the table. For example for each workflow type there is a
    // specific asset requirements (i.e. cam-imu requires a cam, target and imu), but right now that is not enforced
    // anywhere at the database level. Not doing this offers some flexibility and simplicity, but if there was a nice
    // way to do it we should!
    std::string const signature{hashing::Serialize(assets)};

    WorkflowId id;
    if (auto const result{ReadWorkflowId(db, type, signature)}) {
        id = *result;
    } else {
        id = InsertWorkflowId(db, type, signature);
    }

    // NOTE(Jack): This is a unique component of the workflow creation that we do not find in the step or asset
    // "GetOrCreate" methods.
    WorkflowAssetsInsert(db, id, assets);

    return id;
}

void WorkflowAssetsInsert(sqlite3* const db, WorkflowId const workflow_id, std::vector<AssetId> const& asset_ids) {
    auto const binder{[workflow_id](sqlite3_stmt* const stmt, auto const& asset_id) {
        Bind(stmt, 1, workflow_id.value);
        Bind(stmt, 2, asset_id.value);
    }};

    BatchExecuteStatement(sql_statements::workflow_assets_insert, asset_ids, binder, db);
}

void WorkflowStepUpsert(sqlite3* const db, WorkflowId const workflow_id, StepType const step_type,
                        StepId const step_id) {
    auto const binder{[workflow_id, step_type, step_id](sqlite3_stmt* const stmt) {
        // NOTE(Jack): Including the step type here enforces our constraint that a workflow can only have one of each
        // kind of step. This might get annoying for certain things like reprojection error calculation steps which now
        // will all need unique step types. But unless we combine those with their originating steps like we had in v1
        // that was always going to be a problem.
        Bind(stmt, 1, workflow_id.value);
        Bind(stmt, 2, ToString(step_type));
        Bind(stmt, 3, step_id.value);
    }};

    ExecuteStatement(sql_statements::workflow_steps_upsert, binder, db);
}

// TODO(Jack): The semantics are confusing because while the cache_key is passed here it is never written into the
// step, it is only used to check for a cache hit or not. To actually write the cache key to the db you need to call
// StepCacheKeyUpdate.
std::pair<StepId, CacheStatus> GetOrCreateStep(sqlite3* const db, StepType const type, Hash const& cache_key) {
    auto const result{ReadStepId(db, type, cache_key)};
    if (result.has_value()) {
        return std::make_pair(*result, CacheStatus::CacheHit);
    }

    return std::make_pair(InsertStep(db, type), CacheStatus::CacheMiss);
}

// TODO(Jack): How do we handle the case when we are asked to complete a step which does not exist? Throw? Does it
// already do that?
void StepCacheKeyUpdate(sqlite3* const db, StepId const step_id, Hash const& cache_key) {
    auto const binder{[step_id, cache_key](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, cache_key.value);
        Bind(stmt, 2, step_id.value);
    }};

    ExecuteStatement(sql_statements::steps_update_cache_key, binder, db);
}

void CameraInfoInsert(sqlite3* const db, StepId const step_id, AssetId const asset_id, CameraInfo const& camera_info) {
    auto const binder{[step_id, asset_id, camera_info](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, ToString(camera_info.camera_model));
        Bind(stmt, 4, static_cast<int64_t>(camera_info.bounds.v_max));
        Bind(stmt, 5, static_cast<int64_t>(camera_info.bounds.u_max));
    }};

    ExecuteStatement(sql_statements::camera_info_insert, binder, db);
}

std::expected<CameraInfo, std::string> CameraInfoSelect(sqlite3* const db, StepId const step_id,
                                                        AssetId const asset_id) {
    std::optional<CameraInfo> camera_info{std::nullopt};

    ExecuteQuery(
        db, sql_statements::camera_info_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&camera_info](sqlite3_stmt* const stmt) {
            CameraInfo result;
            result.camera_model = ToCameraModel(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0)));
            result.bounds.v_max = sqlite3_column_int(stmt, 1);
            result.bounds.v_min = 0;
            result.bounds.u_max = sqlite3_column_int(stmt, 2);
            result.bounds.u_min = 0;

            camera_info = result;
        });

    if (camera_info) {
        return *camera_info;
    } else {
        return std::unexpected(std::format("{{'database::': '{}', 'step_id': {}, 'asset_id': {}}}", "CameraInfoSelect",
                                           step_id.value, asset_id.value));
    }
}

void CameraPosesInsert(sqlite3* const db, StepId const step_id, StepId source_step_id, AssetId const asset_id,
                       Frames const& camera_poses) {
    auto const binder{[step_id, source_step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, frame] = data_i;

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, source_step_id.value);
        Bind(stmt, 3, asset_id.value);
        Bind(stmt, 4, timestamp_ns);
        BindEigenColumn<Array6d>(stmt, 5, frame.pose);
    }};

    BatchExecuteStatement(sql_statements::camera_poses_insert, camera_poses, binder, db);
}

Frames CameraPosesSelect(sqlite3* const db, StepId step_id, AssetId asset_id) {
    Frames data;

    ExecuteQuery(
        db, sql_statements::camera_poses_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
            Array6d const loaded{ReadEigenColumn<6>(stmt, 1)};

            data.insert(Frame{timestamp_ns, loaded});
        });

    return data;
}  // LCOV_EXCL_LINE

void ControlPointsInsert(sqlite3* const db, StepId const step_id, AssetId const asset_id,
                         spline::Matrix2NXd const& data) {
    // NOTE(Jack): This lets use treat the columns of the eigen matrix like a regular type that we can iterate over.
    // This is required to be compatible with BatchExecuteStatement().
    auto indexed_control_point_columns{[](auto const& control_points) {
        return std::views::iota(0, static_cast<int>(control_points.cols())) |
               std::views::transform(
                   [&control_points](int const i) { return std::pair{i, Array6d{control_points.col(i)}}; });
    }};

    auto const binder{[step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [i, control_point]{data_i};

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, static_cast<int64_t>(i));
        BindEigenColumn<Array6d>(stmt, 4, control_point);
    }};

    BatchExecuteStatement(sql_statements::control_points_insert, indexed_control_point_columns(data), binder, db);
}

spline::Matrix2NXd ControlPointsSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    std::vector<Eigen::Matrix<double, 6, 1>> points;
    ExecuteQuery(
        db, sql_statements::control_points_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&points](sqlite3_stmt* stmt) { points.emplace_back(ReadEigenColumn<6>(stmt, 1)); });

    // NOTE(Jack): This only works because in the sql statement we load them ordered by the idx (ex. ORDER BY idx
    // ASC). If we did not do that then we would need to use the idx itself to place the control points in the right
    // column.
    spline::Matrix2NXd control_points(6, std::size(points));
    for (size_t i{0}; i < std::size(points); ++i) {
        control_points.col(i) = points[i];
    }

    return control_points;
}

// NOTE(Jack): This "source_step_id" idea here is an important part of establishing a foreign key relationship
// between two data tables.
void ExtractedTargetsInsert(sqlite3* const db, StepId const step_id, StepId const source_step_id,
                            AssetId const asset_id, CameraMeasurements const& data) {
    auto const binder{[step_id, source_step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, target]{data_i};

        protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error(  // LCOV_EXCL_LINE
                std::format("ExtractedTargetProto.SerializeToString() failed: step_id '{}', source_step_id '{}', "
                            "asset_id '{}', timestamp_ns '{}'",
                            step_id.value, source_step_id.value, asset_id.value, timestamp_ns));  // LCOV_EXCL_LINE
        }

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, source_step_id.value);
        Bind(stmt, 3, asset_id.value);
        Bind(stmt, 4, timestamp_ns);
        BindBlob(stmt, 5, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::extracted_targets_insert, data, binder, db);
}

CameraMeasurements ExtractedTargetsSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    CameraMeasurements data;

    ExecuteQuery(
        db, sql_statements::extracted_targets_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{SqliteBlob(stmt, 1)};
            protobuf_serialization::ExtractedTargetProto serialized;
            serialized.ParseFromArray(std::data(blob), static_cast<int>(std::size(blob)));

            auto const deserialized{Deserialize(serialized)};
            if (not deserialized) {
                throw std::runtime_error(std::format(  // LCOV_EXCL_LINE
                    "ExtractedTargetProto.ParseFromArray()/Deserialize() failed: "
                    "timestamp_ns '{}'",  // LCOV_EXCL_LINE
                    timestamp_ns));       // LCOV_EXCL_LINE
            }

            data.insert({timestamp_ns, deserialized.value()});
        });

    return data;
}  // LCOV_EXCL_LINE

void ExtrinsicInsert(sqlite3* const db, StepId step_id, Extrinsic const& extrinsic) {
    auto const binder{[step_id, extrinsic](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, extrinsic.frame_a.value);
        Bind(stmt, 3, extrinsic.frame_b.value);
        BindEigenColumn<Array6d>(stmt, 4, extrinsic.se3_a_b);
    }};

    ExecuteStatement(sql_statements::extrinsics_insert, binder, db);
}

std::expected<Extrinsic, std::string> ExtrinsicSelect(sqlite3* const db, StepId const step_id, AssetId const asset_a_id,
                                                      AssetId const asset_b_id) {
    std::optional<Array6d> se3_a_b{std::nullopt};

    ExecuteQuery(
        db, sql_statements::extrinsics_select,
        [step_id, asset_a_id, asset_b_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_a_id.value);
            Bind(stmt, 3, asset_b_id.value);
        },
        [&se3_a_b](sqlite3_stmt* const stmt) { se3_a_b = ReadEigenColumn<6>(stmt, 0); });

    if (se3_a_b) {
        return Extrinsic{asset_a_id, asset_b_id, *se3_a_b};
    } else {
        return std::unexpected(std::format("{{'database::': '{}', 'step_id': {}, 'asset_a_id': {}, 'asset_b_id': {}}}",
                                           "ExtrinsicSelect", step_id.value, asset_a_id.value, asset_b_id.value));
    }
}

// TODO(Jack): Should gravity be assocaited with any asset or any other piece of information? Currently the way we
// store it we have no idea about what frame its in or anything else besides which step it comes from.
void GravityInsert(sqlite3* const db, StepId const step_id, Vector3d const& gravity) {
    auto const binder{[step_id, gravity](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        BindEigenColumn<Vector3d>(stmt, 2, gravity);
    }};

    ExecuteStatement(sql_statements::gravity_insert, binder, db);
}

std::expected<Vector3d, std::string> GravitySelect(sqlite3* const db, StepId const step_id) {
    std::optional<Vector3d> gravity;

    ExecuteQuery(
        db, sql_statements::gravity_select, [step_id](sqlite3_stmt* const stmt) { Bind(stmt, 1, step_id.value); },
        [&gravity](sqlite3_stmt* const stmt) { gravity = ReadEigenColumn<3>(stmt, 0); });

    if (gravity) {
        return *gravity;
    } else {
        return std::unexpected(std::format("{{'database::': '{}', 'step_id': {}}}", "GravitySelect", step_id.value));
    }
}

void ImagesInsert(sqlite3* const db, StepId const step_id, AssetId const asset_id, EncodedImages const& data) {
    auto const binder{[step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, buffer]{data_i};

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, timestamp_ns);

        if (buffer.data.empty()) {
            BindNull(stmt, 4);
        } else {
            BindBlob(stmt, 4, std::as_bytes(std::span{buffer.data}));
        }
    }};

    BatchExecuteStatement(sql_statements::images_insert, data, binder, db);
}

EncodedImages ImagesSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    EncodedImages data;

    ExecuteQuery(
        db, sql_statements::images_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{SqliteBlob(stmt, 1)};
            std::span<uchar const> blob_span{reinterpret_cast<uchar const*>(blob.data()), blob.size()};
            std::vector<uchar> buffer(std::cbegin(blob_span), std::cend(blob_span));

            // TODO(Jack): Should we represent empty images with std::optional? Currently this will load all images,
            // and if the image is a null value it will just be a buffer with length zero.
            data.insert({timestamp_ns, ImageBuffer{buffer}});
        });

    return data;
}  // LCOV_EXCL_LINE

void ImuDataInsert(sqlite3* const db, StepId step_id, AssetId asset_id, ImuMeasurements const& data) {
    auto const binder{[step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, imu_data_i]{data_i};

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, timestamp_ns);

        BindEigenColumn<Vector3d>(stmt, 4, imu_data_i.angular_velocity);
        BindEigenColumn<Vector3d>(stmt, 7, imu_data_i.linear_acceleration);
    }};

    BatchExecuteStatement(sql_statements::imu_data_insert, data, binder, db);
}

ImuMeasurements ImuDataSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    ImuMeasurements data;

    ExecuteQuery(
        db, sql_statements::imu_data_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
            Array6d const imu_data_i{ReadEigenColumn<6>(stmt, 1)};

            data.insert(ImuMeasurement{timestamp_ns, {imu_data_i.topRows<3>(), imu_data_i.bottomRows<3>()}});
        });

    return data;
}  // LCOV_EXCL_LINE

void IntrinsicInsert(sqlite3* const db, StepId const step_id, AssetId const asset_id, CameraModel const camera_model,
                     CameraState const& data) {
    auto const binder{[step_id, asset_id, camera_model, data](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, ToString(camera_model));
        Bind(stmt, 4, ToToml(camera_model, data.intrinsics));
    }};

    ExecuteStatement(sql_statements::intrinsics_insert, binder, db);
}

std::optional<CameraState> IntrinsicSelect(sqlite3* const db, StepId step_id, AssetId asset_id) {
    std::optional<CameraState> intrinsic;

    ExecuteQuery(
        db, sql_statements::intrinsics_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&intrinsic](sqlite3_stmt* const stmt) {
            CameraModel const camera_model{
                ToCameraModel(std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0))))};

            CameraState const result{
                FromToml(camera_model, std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1))))};
            intrinsic = result;
        });

    return intrinsic;
}  // LCOV_EXCL_LINE

void ReprojectionErrorsInsert(sqlite3* const db, StepId const step_id, StepId const source_step_id,
                              AssetId const asset_id, ReprojectionErrors const& data) {
    auto const binder{[step_id, source_step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, reprojection_error] = data_i;

        protobuf_serialization::ArrayX2dProto const serialized{Serialize(reprojection_error)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error(  // LCOV_EXCL_LINE
                std::format("ArrayX2dProto.SerializeToString() failed: step_id '{}', source_step_id '{}', "
                            "asset_id '{}', timestamp_ns '{}'",
                            step_id.value, source_step_id.value, asset_id.value, timestamp_ns));  // LCOV_EXCL_LINE
        }

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, source_step_id.value);
        Bind(stmt, 3, asset_id.value);
        Bind(stmt, 4, timestamp_ns);
        BindBlob(stmt, 5, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::reprojection_errors_insert, data, binder, db);
}

void SplineInfoInsert(sqlite3* const db, StepId step_id, AssetId asset_id, spline::TimeHandler const& time_handler) {
    auto const binder{[step_id, asset_id, time_handler](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        // TODO(Jack): At this time we only support (and therefore hardcode) pose splines. In the future we will add
        // the bias splines for the imu and this will actually become relevant.
        Bind(stmt, 3, "pose");
        Bind(stmt, 4, time_handler.t0_ns_);
        Bind(stmt, 5, time_handler.delta_t_ns_);
    }};

    ExecuteStatement(sql_statements::spline_info_insert, binder, db);
}

// TODO(Jack): Does adding a real SplineInfo struct make sense? Would that simplify some of the complexity we had
// while trying to unify the spline representations?
// TODO(Jack): At this point we made this function/concept over generic by talking about "spline info" but actually
// only really using the time handler. Once we add a bias spline one day we will need to refactor this code (see
// below).
std::optional<spline::TimeHandler> SplineInfoSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    std::optional<spline::TimeHandler> time_handler;

    ExecuteQuery(
        db, sql_statements::spline_info_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&time_handler](sqlite3_stmt* const stmt) {
            // WARN(Jack): We ignore the spline_type in column 0 for now but once we fully adapt the spline_type
            // concept when we hopefully introduce bias splines we need to add this back!
            uint64_t const t0_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 1))};
            uint64_t const delta_t_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 2))};

            time_handler = spline::TimeHandler{t0_ns, delta_t_ns};
        });

    return time_handler;
}

void TargetInfoInsert(sqlite3* const db, StepId const step_id, AssetId const asset_id, TargetInfo const& target_info) {
    auto const binder{[step_id, asset_id, target_info](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, ToString(target_info.target_type));
        Bind(stmt, 4, static_cast<int64_t>(target_info.height));
        Bind(stmt, 5, static_cast<int64_t>(target_info.width));
        Bind(stmt, 6, target_info.unit_dimension);
        Bind(stmt, 7, static_cast<int64_t>(target_info.asymmetric));
    }};

    ExecuteStatement(sql_statements::target_info_insert, binder, db);
}

std::optional<TargetInfo> TargetInfoSelect(sqlite3* const db, StepId const step_id, AssetId const asset_id) {
    std::optional<TargetInfo> target_info;

    ExecuteQuery(
        db, sql_statements::target_info_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
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

}  // namespace reprojection::database