#include "steps/initialize_calibration.hpp"

#include <optional>

#include "database/calibration_database.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

// TODO(Jack): Why is this in the steps module and not the application module? This is not an actual official step.
CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr const db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    WorkflowType const workflow_type{DetermineWorkflowType(cfg)};
    CalibrationAssets const assets{CreateCalibrationAssets(cfg, db)};

    InsertAssetGroups(workflow_type, assets, db);
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), workflow_type, assets.All())};

    log->info("{{'workflow': {{'type': '{}', 'id': {}}}, 'config': {}}}", ToString(workflow_type), workflow_id.value,
              logging::ToOneLineJson(cfg_table));

    return {cfg.application, assets, workflow_id, workflow_type};
}

// NAMING!
// TODO(Jack): Move this to the calibration module? Or application module?
WorkflowType DetermineWorkflowType(config::Config const& cfg) {
    std::size_t const num_cams{cfg.cameras.size()};

    if (cfg.imu) {
        return WorkflowType::CamImu;
    } else if (num_cams == 1) {
        return WorkflowType::Cam;
    } else {
        return WorkflowType::MultiCam;
    }
}

// NAMING!
// TODO(Jack): Move this to the calibration module? Or application module?
CalibrationAssets CreateCalibrationAssets(config::Config const& cfg, SqlitePtr const db) {
    CalibrationAssets assets;

    // Add all the cameras.
    for (auto const& camera : cfg.cameras) {
        AssetId const camera_id{
            database::GetOrCreateAsset(db.get(), AssetType::Camera, camera.index, camera.sensor_name)};
        assets.cameras.push_back({camera_id, camera});
    }

    // Add the target.
    // TODO(Jack): Add ability to pass/add a name from the user config.
    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "target")};
    assets.target = {target_id, cfg.target};

    // Add the imu if it exists.
    if (cfg.imu) {
        AssetId const imu_id{database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, Name{cfg.imu->sensor_name})};
        assets.imu = {imu_id, *cfg.imu};
    }

    return assets;
}

// NAMING!
void InsertAssetGroups(WorkflowType const workflow_type, CalibrationAssets const& assets, SqlitePtr const db) {
    // The workflow asset group contains all the assets in one.
    database::AssetGroupInsert(db.get(), assets.All());

    // Each individual asset is also added - most steps are just owned by one single asset.
    // TODO(Jack): Here we see the problem of the naming - "asset groups" would lead the normal person to believe there
    // is probably at least two or more assets in any group, but that is not true! We need a name that reflects better
    // it is simple a unique asset based identifier for worklow components.
    for (auto const& asset : assets.All()) {
        database::AssetGroupInsert(db.get(), {asset});
    }

    // Now execute any special rules that exist depending on the workflow type.
    if (workflow_type == WorkflowType::CamImu) {
        // WARN(Jack): We are hardcoding here that the imu will always be calibrated to the first camera.
        database::AssetGroupInsert(db.get(), {assets.cameras.at(0).id, assets.imu->id});
    }
}

}  // namespace reprojection::steps
