#include "steps/initialize_calibration.hpp"

#include <optional>

#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

// TODO(Jack): Why is this in the steps module and not the application module? This is not an actual official step.
// TODO(Jack): This is at most capable of initializing a camera-imu extrinsic calibration. If we want a different
// workflow (ex. stereo of multi-cam-imu then we need to refactor this).
CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr const db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    // TODO(Jack): We should refactor our config file so that it can load multiple cameras from one file and
    // automatically assign the index from the config file structure instead of harcoding it to 0 here. Same for the
    // targets and imus below.
    // TODO(Jack): Refactor so user can pass in a target name if they like.
    AssetId const camera_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, cfg.camera.sensor_name)};
    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "aprilgrid")};

    // TODO(Jack): Right now the workflow creation/parameterization here is extremely manual and hardcoded for our two
    // sensor cam/cam-imu only case, but we don't need to complicate things till later when we need a more generic
    // config parsing and workflow creation.
    database::WorkflowType workflow_type{database::WorkflowType::Cam};
    std::vector<AssetId> workflow_assets{camera_id, target_id};

    std::optional<AssetId> imu_id;
    if (cfg.imu) {
        imu_id = database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, Name{cfg.imu->sensor_name});
        workflow_type = database::WorkflowType::CamImu;
        workflow_assets.push_back(*imu_id);
    }

    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), workflow_type, workflow_assets)};

    log->info("{{'workflow_id': '{}', 'assets': {{'camera_id': {}, 'target_id': {}, 'imu_id': {}}}, 'config': {}}}",
              workflow_id.value, camera_id.value, target_id.value, imu_id ? imu_id->value : -1,
              logging::ToOneLineJson(cfg_table));

    return {cfg, workflow_id, camera_id, target_id, imu_id};
}

}  // namespace reprojection::steps
