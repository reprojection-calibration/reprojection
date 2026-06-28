#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "config/config_parse.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/extrinsic_initialization.hpp"
#include "steps/extrinsic_optimization.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/imu_data_loading.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/pose_initialization.hpp"
#include "steps/spline_initialization.hpp"
#include "steps/step_runner.hpp"
#include "steps/target_info.hpp"

#include "io.hpp"
#include "utils.hpp"

namespace reprojection::application {

namespace {

auto const log{logging::Get("application")};

}

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]) {
    auto const paths{ParseCommandLineInput(argc, argv)};
    if (not paths) {
        return std::nullopt;
    }

    auto const config{LoadConfig(paths->config_path)};
    if (not config) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    auto const db{Open(paths->workspace_dir, paths->data_path)};
    if (not db) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    return AppArgs{paths->data_path, *config, *db};
}

// TODO(Jack): Should we put image_source_signature and image_source into one object? They are 100% related. Or maybe
// the entire "image source" concept needs to be reworked as it does not play nice with the database. See note in
// update_feature_extraction_cache_key.py
void Calibrate(toml::table const& config, ImageSourceSignature image_source, std::string const& image_source_signature,
               SqlitePtr const db) {
    // TODO(Jack): Add a config parsing step where all entities get written to the entity table.
    // NOTE(Jack): The entity holds a special place in the calibration process in that it is not part of the regular
    // cachable step workflow process. That is readily apparent by the foreign key dependency of the calibration step
    // table on the entity ids found in the entity table. The most important implication of this is that the entity
    // table needs a custom read/write step like object. And in addition to that, instead of pure "insert" semantics
    // the entity use "insert or ignore" semantics because every time the calibration runs it will try to insert
    // all the entities that are present. And if it is already present that is not an error!
    std::string const camera_name{config["camera"]["sensor_name"].as_string()->get()};
    database::InsertEntity(db, camera_name, Entity::Camera);

    steps::ImageLoading const image_loading{camera_name, image_source_signature, image_source};
    auto const [encoded_images,
                image_loading_cache_status]{steps::RunStep<std::shared_ptr<EncodedImages>>(image_loading, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'encoded_images': {}}}", ToString(image_loading.StepType()),
              ToString(image_loading_cache_status), encoded_images->size());

    steps::CameraInfoStep const camera_info_step{*config["camera"].as_table(), encoded_images};
    auto const [camera_info, ci_cache_status]{steps::RunStep<CameraInfo>(camera_info_step, db)};
    log->info(
        "{{'step': '{}', 'cache_status': '{}', 'sensor_name': {}, 'camera_model': '{}', 'height': {}, 'width': {}}}",
        ToString(camera_info_step.StepType()), ToString(ci_cache_status), camera_info.sensor_name,
        ToString(camera_info.camera_model), camera_info.bounds.v_max, camera_info.bounds.u_max);

    steps::TargetInfoStep const target_info_step{*config["target"].as_table(), camera_info.sensor_name};
    auto const [target_info, target_info_cache_status]{steps::RunStep<TargetInfo>(target_info_step, db)};
    log->info(
        "{{'step': '{}', 'cache_status': '{}', 'target_type': {}, 'height': {}, 'width': {}, 'unit_dimension': {}, "
        "'asymmetric': {}}}",
        ToString(target_info_step.StepType()), ToString(target_info_cache_status), ToString(target_info.target_type),
        target_info.height, target_info.width, target_info.unit_dimension, target_info.asymmetric);

    // TODO(Jack): The loading and parsing of the app config belongs in its own step! Having this here is a hack for
    // now.
    bool show_extraction{true};
    if (auto const node{config["application"]["show_extraction"]}) {
        show_extraction = node.as_boolean()->get();  // LCOV_EXCL_LINE
    }

    steps::FeatureExtraction const feature_extraction_step{camera_info.sensor_name, encoded_images, target_info,
                                                           show_extraction};
    auto const [targets,
                feature_extraction_cache_status]{steps::RunStep<CameraMeasurements>(feature_extraction_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'extracted_targets': {}}}",
              ToString(feature_extraction_step.StepType()), ToString(feature_extraction_cache_status),
              std::size(targets));

    steps::IntrinsicInitialization const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{steps::RunStep<CameraState>(ii_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'intrinsics': {}}}", ToString(ii_step.StepType()),
              ToString(ii_cache_status), camera_state.intrinsics);

    steps::PoseInitialization const pose_init_step{camera_info, targets, camera_state};
    auto const [initial_poses, pose_init_cache_status]{steps::RunStep<Frames>(pose_init_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}}}", ToString(pose_init_step.StepType()),
              ToString(pose_init_cache_status), std::size(initial_poses));

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    steps::BundleAdjustment const ba_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, ba_cache_status]{steps::RunStep<OptimizationState>(ba_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}, 'intrinsics': {}}}", ToString(ba_step.StepType()),
              ToString(ba_cache_status), std::size(initial_poses), optimized_state.camera_state.intrinsics);

    // TODO(Jack): This is a hack! At this moment this is meant for internal development only therefore we will not
    // expose an imu data lambda or add the IMU config sections to the config validation logic. This means that if
    // someone tried to use this from an application it will be impossible.
    // TODO(Jack): Remove code coverage exclusion!
    // LCOV_EXCL_START
    if (config.contains("imu")) {
        auto const result{config::Config::Imu::Parse(*config["imu"].as_table())};
        if (std::holds_alternative<TomlErrorMsg>(result)) {
            throw std::runtime_error{"WE NEED AN ERROR HANDLING STRATEGY!"};  // LCOV_EXCL_LINE
        }

        std::string const imu_name{std::get<config::Config::Imu>(result).sensor_name};

        std::cout << "Doing an IMU calibration... development mode only!" << std::endl;
        database::InsertEntity(db, imu_name, Entity::Imu);
        database::InsertEntity(db, Extrinsic::EntityId(imu_name, camera_info.sensor_name), Entity::Extrinsic);

        // TODO(Jack): One day, when we are done hacking, we will pass in the real serialized data and imu data
        // source lambda! For now though this only works if we write the imu data seperately and manually trigger a
        // cache hit.
        steps::ImuDataLoading const imu_data_loading{imu_name, "", {}};
        auto const [imu_data, imu_data_loading_cache_status]{steps::RunStep<ImuMeasurements>(imu_data_loading, db)};
        log->info("{{'step': '{}', 'cache_status': '{}', 'imu_data': {}}}", ToString(imu_data_loading.StepType()),
                  ToString(imu_data_loading_cache_status), std::size(imu_data));

        steps::SplineInitialization const spline_init_step{camera_info, targets, aligned_initial_state};
        auto const [spline_init, spline_init_cache_status]{steps::RunStep<spline::Se3Spline>(spline_init_step, db)};
        log->info("{{'step': '{}', 'cache_status': '{}', 'num_control_points': {}}}",
                  ToString(spline_init_step.StepType()), ToString(spline_init_cache_status), spline_init.Size());

        steps::ExtrinsicInitialization const extrinsic_init_step{imu_name, camera_info.sensor_name, imu_data,
                                                                 spline_init};
        auto const [extrinsic_init,
                    extrinsic_init_cache_status]{steps::RunStep<ImuCamExtrinsic>(extrinsic_init_step, db)};
        log->info("{{'step': '{}', 'cache_status': '{}'}}", ToString(extrinsic_init_step.StepType()),
                  ToString(extrinsic_init_cache_status));

        std::cout << geometry::Exp(extrinsic_init.tf.se3_a_b).matrix() << std::endl;
        std::cout << extrinsic_init.gravity.transpose() << std::endl;

        steps::ExtrinsicOptimization const extrinsic_opt_step{camera_info, targets,     optimized_state.camera_state,
                                                              imu_data,    spline_init, extrinsic_init};
        auto const [extrinsic_opt_result, extrinsic_opt_cache_status]{
            steps::RunStep<std::pair<spline::Se3Spline, ImuCamExtrinsic>>(extrinsic_opt_step, db)};
        log->info("{{'step': '{}', 'cache_status': '{}'}}", ToString(extrinsic_opt_step.StepType()),
                  ToString(extrinsic_opt_cache_status));

        std::cout << geometry::Exp(extrinsic_opt_result.second.tf.se3_a_b).matrix() << std::endl;
        std::cout << extrinsic_opt_result.second.gravity.transpose() << std::endl;
    }
    // LCOV_EXCL_STOP
}

}  // namespace reprojection::application
