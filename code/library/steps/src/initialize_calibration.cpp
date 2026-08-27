#include "steps/initialize_calibration.hpp"

#include <optional>

#include "database/calibration_database.hpp"
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

    // TODO(Jack): Refactor so user can pass in a target name if they like.
    AssetId const camera_id{1};
    database::AssetGroupInsert(db.get(), {camera_id});

    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "target")};
    database::AssetGroupInsert(db.get(), {target_id});

    // TODO(Jack): Right now the workflow creation/parameterization here is extremely manual and hardcoded for our two
    // sensor cam/cam-imu only case, but we don't need to complicate things till later when we need a more generic
    // config parsing and workflow creation.
    WorkflowType workflow_type{WorkflowType::Cam};
    std::vector<AssetId> workflow_assets{camera_id, target_id};

    std::optional<AssetId> imu_id;
    if (cfg.imu) {
        imu_id = database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, Name{cfg.imu->sensor_name});
        database::AssetGroupInsert(db.get(), {*imu_id});
        database::AssetGroupInsert(db.get(), {*imu_id, camera_id});

        workflow_type = WorkflowType::CamImu;
        workflow_assets.push_back(*imu_id);
    }

    database::AssetGroupInsert(db.get(), {workflow_assets});
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), workflow_type, workflow_assets)};

    log->info(
        "{{'workflow': {{'type': '{}', 'id': {}}}, 'assets': {{'camera_id': {}, 'target_id': {}, 'imu_id': {}}}, "
        "'config': {}}}",
        ToString(workflow_type), workflow_id.value, camera_id.value, target_id.value, imu_id ? imu_id->value : -1,
        logging::ToOneLineJson(cfg_table));

    return {};
}

// NAMING!
// TODO(Jack): Move this to the calibration module? Or application module?
WorkflowType DetermineWorkflowType(config::Config const& cfg) {
    std::size_t const num_cams{cfg.cameras.size()};

    if (cfg.imu) {
        if (num_cams == 1) {
            return WorkflowType::CamImu;
        }

        throw std::runtime_error(
            std::format("Camera-IMU calibration currently requires exactly one camera, got {}", num_cams));
    }

    if (num_cams == 1) {
        return WorkflowType::Cam;
    } else if (num_cams == 2) {
        return WorkflowType::MultiCam;
    }

    throw std::runtime_error(
        std::format("Multi-camera calibration currently supports exactly two cameras, got {}", num_cams));
}

// NAMING!
// TODO(Jack): Move this to the calibration module? Or application module?
CalibrationAssets CreateCalibrationAssets(config::Config const& cfg, SqlitePtr const db) {
    CalibrationAssets assets;

    // Add all the cameras.
    for (auto const& camera : cfg.cameras) {
        AssetId const camera_id{
            database::GetOrCreateAsset(db.get(), AssetType::Camera, camera.index, camera.sensor_name)};
        assets.cameras.push_back(camera_id);

        database::AssetGroupInsert(db.get(), {camera_id});
    }

    // Add the target.
    assets.target = database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "target");
    database::AssetGroupInsert(db.get(), {assets.target});

    // Add the imu if it exists.
    if (cfg.imu) {
        assets.imu = database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, Name{cfg.imu->sensor_name});
        database::AssetGroupInsert(db.get(), {*assets.imu});
    }

    return assets;
}

}  // namespace reprojection::steps
