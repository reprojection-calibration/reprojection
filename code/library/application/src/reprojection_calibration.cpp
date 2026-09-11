#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "config/config_parse.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/imu_data_loading.hpp"
#include "steps/initialize_workflow.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/pose_initialization.hpp"
#include "steps/spline_init.hpp"
#include "steps/step_runner.hpp"
#include "steps/stereo_rig_init.hpp"
#include "steps/stereo_rig_opt.hpp"
#include "steps/target_info.hpp"
#include "steps/visual_inertial_init.hpp"
#include "steps/visual_inertial_opt.hpp"

#include "io.hpp"

namespace reprojection::application {

// NOTE(Jack): The logging colors! Having the terminal log all be one single color made it a little hard to interpret
// sometimes. My preferred solution would have been specifying the color when we initiate the logger in each file but
// that wasn't super obvious to me and didnt work at the first try. This would be the dream solution:
//
//      auto const log{logging::Get("application", LogColor::Magenta)};
//
// And then all the log statements in this file that use that logger would be magenta. Our temporary solution because we
// could not get that to work is really quite hacky and involves putting in some formatting color codes directly into
// the log message. This is hacky because if for example you write these logs to file the color codes will actually be
// in the log... that is not so nice. In our application the implementation looks something like:
//
//      log->info("\033[35m blah \033[0m", camera);
//
// Will log 'blah' in magenta. The first code turns on the color and the end code resets the color. The following colors
// that we used so far (11.09.2026) are:
//
//      \033[32m - green
//      \033[34m - blue
//      \033[35m - magenta
//      \033[0m - reset
//
// The only reason we are tolerating such a hard coded hacky (and potentially not portable/clean) solution is because
// its high impact to the user and we only had to apply it in three or four log statements. If it grows beyond that then
// we need an official solution as I described at the start of this note.

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
    std::ranges::transform(cfg.cameras, std::back_inserter(camera_names),
                           [](auto const& camera) { return camera.sensor_name; });

    std::optional<std::string> imu_name{std::nullopt};
    if (cfg.imu) {
        imu_name = cfg.imu->sensor_name;
    }

    return Sensors{camera_names, imu_name};
}

std::vector<CamStageIds> CamStages(steps::CalibrationContext const& context, StepId const& target_info_id,
                                   ImageInputs const& image_inputs, SqlitePtr const db) {
    std::vector<CamStageIds> camera_calibrations;
    for (auto const& camera : context.assets.cameras) {
        log->info("\033[35m{{'stage': 'single_cam', 'asset': {}}}\033[0m", camera);

        ImageInput const& image_input{image_inputs.at(camera.config.sensor_name)};

        steps::ImageLoading const image_loading_step{camera.id, image_input.signature, image_input.source};
        StepId const image_loading_id{steps::RunStep<steps::ImageLoading>(context.workflow_id, image_loading_step, db)};

        steps::CameraInfoStep const camera_info_step{camera.id, image_loading_id, camera.config.camera_model, db};
        StepId const camera_info_id{RunStep<steps::CameraInfoStep>(context.workflow_id, camera_info_step, db)};

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

    return camera_calibrations;
}

void Calibrate(toml::table const& cfg_table, ImageInputs const& image_inputs, std::optional<ImuInput> const& imu_input,
               SqlitePtr const db) {
    steps::CalibrationContext const context{steps::InitializeCalibration(cfg_table, db)};

    // Only one target is allowed.
    steps::TargetInfoStep const target_info_step{context.assets.target.id, context.assets.target.config};
    StepId const target_info_id{RunStep<steps::TargetInfoStep>(context.workflow_id, target_info_step, db)};

    std::vector const cam_stages{CamStages(context, target_info_id, image_inputs, db)};

    // TODO USE THE SAME REFERENCE CAMERA FOR THE MULTICAM STEPS!
    // TODO(Jack): What is better, 'cam0' or 'reference_cam'?
    // NOTE(Jack): We arbitrarily choose the first camera as the reference camera. This is an open point!
    auto const& cam0{cam_stages.front()};

    bool const is_multicam{std::size(context.assets.cameras) > 1};
    if (is_multicam) {
        log->info("\033[35m{{'stage': 'multi_cam', 'assets': {}}}\033[0m", context.assets.cameras);

        steps::StereoRigInit const stereo_rig_init{cam_stages, context.application.approx_sync_delta_ns, db};
        StepId const stereo_rig_init_id{RunStep<steps::StereoRigInit>(context.workflow_id, stereo_rig_init, db)};

        steps::StereoRigOpt const stereo_rig_opt{cam_stages, stereo_rig_init_id, context.application.threads,
                                                 context.application.approx_sync_delta_ns, db};
        StepId const stereo_rig_opt_id{RunStep<steps::StereoRigOpt>(context.workflow_id, stereo_rig_opt, db)};

        // TODO(Jack): Should we be using the rig poses here for the cam-imu extrinsic calibration? I think so.
        static_cast<void>(stereo_rig_opt_id);
    }

    // TODO(Jack): Find a way to get this to run in a unit test! I think we could do this with the data generation
    // functions we have!
    // LCOV_EXCL_START

    bool const has_imu{context.assets.imu.has_value() && imu_input.has_value()};
    if (has_imu) {
        auto const imu_id{context.assets.imu->id};
        // WARN(Jack): Again hardcoding here that the first camera is the reference camera!
        log->info("\033[35m{{'stage': 'cam_imu', 'assets': [{}, {{'sensor_name': '{}', 'asset_id': {}}}]}}\033[0m",
                  *context.assets.cameras.begin(), context.assets.imu->config.sensor_name, imu_id.value);

        steps::ImuDataLoading const imu_data_loading_step{imu_id, imu_input->signature, imu_input->source};
        StepId const imu_data_id{steps::RunStep<steps::ImuDataLoading>(context.workflow_id, imu_data_loading_step, db)};

        steps::SplineInit const spline_init_step{cam0, db};
        StepId const spline_init_id{steps::RunStep<steps::SplineInit>(context.workflow_id, spline_init_step, db)};

        steps::VisualInertialInit const visual_inertial_init{
            cam0.asset_id, spline_init_id, imu_id, imu_data_id, context.application.threads, db};
        StepId const visual_inertial_init_id{
            steps::RunStep<steps::VisualInertialInit>(context.workflow_id, visual_inertial_init, db)};

        steps::VisualInertialOpt const visual_inertial_opt_step{cam0.asset_id,
                                                                imu_id,
                                                                cam0.targets_id,
                                                                imu_data_id,
                                                                context.application.threads,
                                                                cam0.camera_info_id,
                                                                cam0.bundle_adjustment_id,
                                                                spline_init_id,
                                                                visual_inertial_init_id,
                                                                db};
        StepId const visual_inertial_opt_id{
            steps::RunStep<steps::VisualInertialOpt>(context.workflow_id, visual_inertial_opt_step, db)};

        static_cast<void>(visual_inertial_opt_id);
    }

    // LCOV_EXCL_STOP

    std::cout << "The future is calibrated!\n";
}

}  // namespace reprojection::application
