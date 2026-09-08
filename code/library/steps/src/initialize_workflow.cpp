#include "steps/initialize_workflow.hpp"

#include <optional>
#include <ranges>

#include "database/calibration_database.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr const db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};
    CalibrationAssets const assets{CreateCalibrationAssets(cfg, db)};
    InsertAssetGroups(assets, db);

    // ERROR(Jack): We hardcode the workflow type here but we should just remove the workflow type entirely!!!
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), WorkflowType::Cam, assets.All())};

    log->info("{{'workflow': {{'id': {}}}, 'config': {}}}", workflow_id.value, logging::ToOneLineJson(cfg_table));

    return {cfg.application, assets, workflow_id};
}

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

void InsertAssetGroups(CalibrationAssets const& assets, SqlitePtr const db) {
    auto const asset_ids{assets.All()};

    // The complete calibration asset group.
    database::AssetGroupInsert(db.get(), asset_ids);

    // Each individual asset.
    for (auto const& asset_id : asset_ids) {
        database::AssetGroupInsert(db.get(), {asset_id});
    }

    // Add all camera pairs - used in cam-cam extrinsic initialization (at minimum).
    for (auto const& camera_id_b : assets.cameras | std::views::drop(1)) {
        // WARN(Jack): Hardcoded importance of the first camera!
        auto const& camera_id_a{assets.cameras.front()};

        database::AssetGroupInsert(db.get(), {camera_id_a.id, camera_id_b.id});
    }

    // The IMU is calibrated relative to the reference camera.
    if (assets.imu.has_value()) {
        // WARN(Jack): Hardcoded importance of the first camera!
        database::AssetGroupInsert(db.get(), {assets.cameras.front().id, assets.imu->id});
    }
}

}  // namespace reprojection::steps
