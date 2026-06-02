#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/pose_initialization.hpp"
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

    auto const config{LoadAndValidateConfig(paths->config_path)};
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
    steps::ImageLoading const image_loading{config["camera"]["sensor_name"].as_string()->get(), image_source_signature,
                                            image_source};
    auto const [encoded_images,
                image_loading_cache_status]{steps::RunStep<std::shared_ptr<EncodedImages>>(image_loading, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'encoded_images': {}}}", ToString(image_loading.step_type),
              ToString(image_loading_cache_status), encoded_images->size());

    steps::CameraInfoStep const camera_info_step{*config["camera"].as_table(), encoded_images};
    auto const [camera_info, ci_cache_status]{steps::RunStep<CameraInfo>(camera_info_step, db)};
    log->info(
        "{{'step': '{}', 'cache_status': '{}', 'sensor_name': {}, 'camera_model': '{}', 'height': {}, 'width': {}}}",
        ToString(camera_info_step.step_type), ToString(ci_cache_status), camera_info.sensor_name,
        ToString(camera_info.camera_model), camera_info.bounds.v_max, camera_info.bounds.u_max);

    steps::TargetInfoStep const target_info_step{*config["target"].as_table(), camera_info.sensor_name};
    auto const [target_info, target_info_cache_status]{steps::RunStep<TargetInfo>(target_info_step, db)};
    log->info(
        "{{'step': '{}', 'cache_status': '{}', 'target_type': {}, 'height': {}, 'width': {}, 'unit_dimension': {}, "
        "'asymmetric': {}}}",
        ToString(target_info_step.step_type), ToString(target_info_cache_status), ToString(target_info.target_type),
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
              ToString(feature_extraction_step.step_type), ToString(feature_extraction_cache_status),
              std::size(targets));

    steps::IntrinsicInitialization const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{steps::RunStep<CameraState>(ii_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'intrinsics': {}}}", ToString(ii_step.step_type),
              ToString(ii_cache_status), camera_state.intrinsics);

    steps::PoseInitialization const pose_init_step{camera_info, targets, camera_state};
    auto const [initial_poses, pose_init_cache_status]{steps::RunStep<Frames>(pose_init_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}}}", ToString(pose_init_step.step_type),
              ToString(pose_init_cache_status), std::size(initial_poses));

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    steps::BundleAdjustment const ba_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, ba_cache_status]{steps::RunStep<OptimizationState>(ba_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}, 'intrinsics': {}}}", ToString(ba_step.step_type),
              ToString(ba_cache_status), std::size(initial_poses), optimized_state.camera_state.intrinsics);
}

}  // namespace reprojection::application
