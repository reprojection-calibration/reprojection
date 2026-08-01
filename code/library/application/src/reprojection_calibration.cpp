#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "config/config_parse.hpp"
#include "logging/logging.hpp"
#include "steps/camera_info.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/initialize_calibration.hpp"
#include "steps/step_runner.hpp"
#include "steps/target_info.hpp"

#include "io.hpp"

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

void Calibrate(toml::table const& cfg_table, ImageInput const& image_input, std::optional<ImuInput> const& imu_input,
               database::CalibrationDatabase& db) {
    steps::CalibrationContext const cfg{steps::InitializeCalibration(cfg_table, db)};

    // TODO(Jack): Clarify the usage of owner! For now we make every stepped owned by the recording
    auto const owner{steps::StepOwner::Recording(cfg.recording_id)};

    steps::ImageLoading const image_loading_step{cfg.camera_id, image_input.signature, image_input.source};
    StepId const image_loading_id{steps::RunStep<steps::ImageLoading>(owner, image_loading_step, db)};

    steps::CameraInfoStep const camera_info_step{cfg.camera_id, image_loading_id, cfg.config.camera.camera_model, db};
    StepId const camera_info_id{RunStep<steps::CameraInfoStep>(owner, camera_info_step, db)};

    steps::TargetInfoStep const target_info_step{cfg.target_id, cfg.config.target};
    StepId const target_info_id{RunStep<steps::TargetInfoStep>(owner, target_info_step, db)};

    steps::FeatureExtraction const step{cfg.camera_id,  image_loading_id, cfg.config.application.show_extraction,
                                        target_info_id, cfg.target_id,    db};
    StepId const feature_extraction_id{RunStep<steps::FeatureExtraction>(owner, step, db)};

    static_cast<void>(imu_input);
    static_cast<void>(camera_info_id);
    static_cast<void>(feature_extraction_id);

    std::cout << "The future is calibrated!\n";
}

}  // namespace reprojection::application
