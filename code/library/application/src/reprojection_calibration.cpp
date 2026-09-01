#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "../../steps/include/steps/initialize_workflow.hpp"
#include "config/config_parse.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/extrinsic_init.hpp"
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

namespace reprojection::application {

namespace {

// We get a name conflict here with some math functions if we just use 'log' like we normally do, so prepend a
// underscore.
auto const log{logging::Get("application")};

}  // namespace

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

// TODO(Jack): To be honest I do not like having this function because now we parse the entire config twice. Once on the
// application side and once on the library side. It is not the end of the world but we should keep our eyes out for any
// hints that we are missing the point.
// TODO(Jack): Should we actually make this a constructor/factory of Sensors? Might just be a better way to organize
// things?
Sensors ParseSensors(toml::table const& cfg_table) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    std::vector<std::string> camera_names;
    for (auto const& camera : cfg.cameras) {
        // cppcheck-suppress useStlAlgorithm
        camera_names.push_back(camera.sensor_name);
    }

    std::optional<std::string> imu_name{std::nullopt};
    if (cfg.imu) {
        imu_name = cfg.imu->sensor_name;
    }

    return Sensors{camera_names, imu_name};
}

// TODO(Jack): Should we move this to another location?
// NOTE(Jack): We only store the values that we need to the extrinsic calibration. We could store every step id but why?
struct CameraCalibration {
    AssetId camera_id;
    StepId camera_info_id;
    StepId targets_id;
    StepId pose_init_id;
    StepId bundle_adjustment_id;
};

void Calibrate(toml::table const& cfg_table, ImageInputs const& image_inputs, std::optional<ImuInput> const& imu_input,
               SqlitePtr const db) {
    steps::CalibrationContext const context{steps::InitializeCalibration(cfg_table, db)};

    std::vector<CameraCalibration> camera_calibrations;
    for (auto const& camera : context.assets.cameras) {
        log->info("{{'sensor_name': '{}', 'asset_id': {}}}", camera.config.sensor_name, camera.id.value);

        ImageInput const& image_input{image_inputs.at(camera.config.sensor_name)};

        steps::ImageLoading const image_loading_step{camera.id, image_input.signature, image_input.source};
        StepId const image_loading_id{steps::RunStep<steps::ImageLoading>(context.workflow_id, image_loading_step, db)};

        steps::CameraInfoStep const camera_info_step{camera.id, image_loading_id, camera.config.camera_model, db};
        StepId const camera_info_id{RunStep<steps::CameraInfoStep>(context.workflow_id, camera_info_step, db)};

        steps::TargetInfoStep const target_info_step{context.assets.target.id, context.assets.target.config};
        StepId const target_info_id{RunStep<steps::TargetInfoStep>(context.workflow_id, target_info_step, db)};

        steps::FeatureExtraction const feature_extraction_step{
            camera.id,      image_loading_id,         context.application.show_extraction,
            target_info_id, context.assets.target.id, db};
        StepId const targets_id{RunStep<steps::FeatureExtraction>(context.workflow_id, feature_extraction_step, db)};

        steps::IntrinsicInitialization const intrinsic_init_step{camera.id, context.application.threads, camera_info_id,
                                                                 targets_id, db};
        StepId const intrinsic_init_id{
            RunStep<steps::IntrinsicInitialization>(context.workflow_id, intrinsic_init_step, db)};

        steps::PoseInitialization const pose_init_step{camera.id, targets_id, camera_info_id, intrinsic_init_id, db};
        StepId const pose_init_id{RunStep<steps::PoseInitialization>(context.workflow_id, pose_init_step, db)};

        steps::BundleAdjustment const bundle_adjustment_step{
            camera.id, targets_id, context.application.threads, camera_info_id, intrinsic_init_id, pose_init_id, db};
        StepId const bundle_adjustment_id{
            RunStep<steps::BundleAdjustment>(context.workflow_id, bundle_adjustment_step, db)};

        camera_calibrations.push_back({camera.id, camera_info_id, targets_id, pose_init_id, bundle_adjustment_id});
    }

    if (context.assets.imu.has_value() and imu_input.has_value()) {
        auto const imu_id{context.assets.imu->id};
        log->info("{{'sensor_name': '{}', 'asset_id': {}}}", context.assets.imu->config.sensor_name, imu_id.value);

        steps::ImuDataLoading const imu_data_loading_step{imu_id, imu_input->signature, imu_input->source};
        StepId const imu_data_id{steps::RunStep<steps::ImuDataLoading>(context.workflow_id, imu_data_loading_step, db)};

        // NOTE(Jack): We arbitrarily choose the first camera as the reference camera. This is an open point!
        auto const& reference_camera{camera_calibrations.front()};

        // ERROR(Jack): Am I crazy or should I not be passing the optimized bundle adjustment poses and not the
        // unrefined pose init poses here? For some reason when I do that the extrinsic init does not work like before,
        // we need to look at this in the debug dashboard and figure out what is going on here. The entire "align
        // rotations" thing play an important part here I think. This is a known problem.
        steps::SplineInitialization const spline_init_step{
            reference_camera.camera_id,      reference_camera.pose_init_id,         reference_camera.targets_id,
            reference_camera.camera_info_id, reference_camera.bundle_adjustment_id, db};
        StepId const spline_init_id{
            steps::RunStep<steps::SplineInitialization>(context.workflow_id, spline_init_step, db)};

        steps::ExtrinsicInit const extrinsic_init_step{
            reference_camera.camera_id, spline_init_id, imu_id, imu_data_id, context.application.threads, db};
        StepId const extrinsic_init_id{
            steps::RunStep<steps::ExtrinsicInit>(context.workflow_id, extrinsic_init_step, db)};

        steps::ExtrinsicOptimization const extrinsic_optimization_step{reference_camera.camera_id,
                                                                       imu_id,
                                                                       reference_camera.targets_id,
                                                                       imu_data_id,
                                                                       context.application.threads,
                                                                       reference_camera.camera_info_id,
                                                                       reference_camera.bundle_adjustment_id,
                                                                       spline_init_id,
                                                                       extrinsic_init_id,
                                                                       db};
        StepId const extrinsic_optimization_id{
            steps::RunStep<steps::ExtrinsicOptimization>(context.workflow_id, extrinsic_optimization_step, db)};

        static_cast<void>(extrinsic_optimization_id);
    }

    std::cout << "The future is calibrated!\n";
}

}  // namespace reprojection::application
