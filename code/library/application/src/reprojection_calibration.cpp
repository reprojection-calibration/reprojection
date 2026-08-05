#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "config/config_parse.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/extrinsic_init.hpp"
#include "steps/extrinsic_optimization.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/imu_data_loading.hpp"
#include "steps/initialize_calibration.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/pose_initialization.hpp"
#include "steps/spline_initialization.hpp"
#include "steps/step_runner.hpp"
#include "steps/target_info.hpp"

#include "io.hpp"

namespace reprojection::application {

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

void Calibrate(toml::table const& cfg_table, ImageInput const& image_input, std::optional<ImuInput> const& imu_input,
               database::CalibrationDatabase const& db) {
    steps::CalibrationContext const cfg{steps::InitializeCalibration(cfg_table, db)};

    steps::ImageLoading const image_loading_step{cfg.camera_id, image_input.signature, image_input.source};
    StepId const image_loading_id{steps::RunStep<steps::ImageLoading>(image_loading_step, db)};

    steps::CameraInfoStep const camera_info_step{cfg.camera_id, image_loading_id, cfg.config.camera.camera_model, db};
    StepId const camera_info_id{RunStep<steps::CameraInfoStep>(camera_info_step, db)};

    steps::TargetInfoStep const target_info_step{cfg.target_id, cfg.config.target};
    StepId const target_info_id{RunStep<steps::TargetInfoStep>(target_info_step, db)};

    steps::FeatureExtraction const feature_extraction_step{
        cfg.camera_id, image_loading_id, cfg.config.application.show_extraction, target_info_id, cfg.target_id, db};
    StepId const targets_id{RunStep<steps::FeatureExtraction>(feature_extraction_step, db)};

    steps::IntrinsicInitialization const intrinsic_init_step{cfg.camera_id, cfg.config.application.threads,
                                                             camera_info_id, targets_id, db};
    StepId const intrinsic_init_id{RunStep<steps::IntrinsicInitialization>(intrinsic_init_step, db)};

    steps::PoseInitialization const pose_init_step{cfg.camera_id, targets_id, camera_info_id, intrinsic_init_id, db};
    StepId const pose_init_id{RunStep<steps::PoseInitialization>(pose_init_step, db)};

    steps::BundleAdjustment const bundle_adjustment_step{
        cfg.camera_id, targets_id, cfg.config.application.threads, camera_info_id, intrinsic_init_id, pose_init_id, db};
    StepId const bundle_adjustment_id{RunStep<steps::BundleAdjustment>(bundle_adjustment_step, db)};

    static_cast<void>(bundle_adjustment_id);

    if (cfg.imu_id.has_value() and imu_input.has_value()) {
        steps::ImuDataLoading const imu_data_loading_step{*cfg.imu_id, imu_input->signature, imu_input->source};
        StepId const imu_data_id{steps::RunStep<steps::ImuDataLoading>(imu_data_loading_step, db)};

        steps::SplineInitialization const spline_init_step{cfg.camera_id, bundle_adjustment_id, db};
        StepId const spline_init_id{steps::RunStep<steps::SplineInitialization>(spline_init_step, db)};

        steps::ExtrinsicInit const extrinsic_init_step{
            cfg.camera_id, spline_init_id, *cfg.imu_id, imu_data_id, cfg.config.application.threads, db};
        StepId const extrinsic_init_id{steps::RunStep<steps::ExtrinsicInit>(extrinsic_init_step, db)};

        steps::ExtrinsicOptimization const extrinsic_optimization_step{
            cfg.camera_id,  *cfg.imu_id,          targets_id,     imu_data_id,       cfg.config.application.threads,
            camera_info_id, bundle_adjustment_id, spline_init_id, extrinsic_init_id, db};
        StepId const extrinsic_optimization_id{
            steps::RunStep<steps::ExtrinsicOptimization>(extrinsic_optimization_step, db)};

        static_cast<void>(extrinsic_optimization_id);
    }

    std::cout << "The future is calibrated!\n";
}

}  // namespace reprojection::application
