#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/camera_info.hpp"
#include "steps/camera_nonlinear_refinement.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/linear_pose_initialization.hpp"
#include "steps/step_runner.hpp"

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
        return std::nullopt;
    }

    auto const db{Open(paths->workspace_dir, paths->data_path)};
    if (not db) {
        return std::nullopt;
    }

    return AppArgs{paths->data_path, *config, *db};
}

// TODO(Jack): Should we put image_source_signature and image_source into one object? They are 100% related. Or maybe
// the entire "image source" concept needs to be reworked as it does not play nice with the database. See note in
// update_feature_extraction_cache_key.py
void Calibrate(toml::table const& config, ImageSource image_source, std::string const& image_source_signature,
               SqlitePtr const db) {
    steps::ImageLoadingStep const image_loading{config["sensor"]["camera_name"].as_string()->get(),
                                                image_source_signature, image_source};
    auto const [encoded_images, il_cache_status]{steps::RunStep<std::shared_ptr<EncodedImages>>(image_loading, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'encoded_images': {}}}", ToString(image_loading.step_type),
              ToString(il_cache_status), encoded_images->size());

    steps::CameraInfoStep const ci_step{*config["sensor"].as_table(), encoded_images};
    auto const [camera_info, ci_cache_status]{steps::RunStep<CameraInfo>(ci_step, db)};
    log->info(
        "{{'step': '{}', 'cache_status': '{}', 'camera_name': {}, 'camera_model': '{}', 'height': {}, 'width': {}}}",
        ToString(ci_step.step_type), ToString(ci_cache_status), camera_info.sensor_name,
        ToString(camera_info.camera_model), camera_info.bounds.v_max, camera_info.bounds.u_max);

    steps::FeatureExtractionStep const ftext_step{camera_info.sensor_name, encoded_images,
                                                  *config["target"].as_table()};
    auto const [targets, ftext_cache_status]{steps::RunStep<CameraMeasurements>(ftext_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'extracted_targets': {}}}", ToString(ftext_step.step_type),
              ToString(ftext_cache_status), std::size(targets));

    steps::IntrinsicInitializationStep const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{steps::RunStep<CameraState>(ii_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'intrinsics': {}}}", ToString(ii_step.step_type),
              ToString(ii_cache_status), camera_state.intrinsics);

    steps::LpiStep const lpi_step{camera_info, targets, camera_state};
    auto const [initial_poses, lpi_cache_status]{steps::RunStep<Frames>(lpi_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}}}", ToString(lpi_step.step_type),
              ToString(lpi_cache_status), std::size(initial_poses));

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    steps::CnlrStep const cnlr_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, cnlr_cache_status]{steps::RunStep<OptimizationState>(cnlr_step, db)};
    log->info("{{'step': '{}', 'cache_status': '{}', 'num_poses': {}, 'intrinsics': {}}}",
              ToString(cnlr_step.step_type), ToString(cnlr_cache_status), std::size(initial_poses),
              optimized_state.camera_state.intrinsics);
}

}  // namespace reprojection::application
