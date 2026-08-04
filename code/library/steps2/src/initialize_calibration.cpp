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
CalibrationContext InitializeCalibration(toml::table const& cfg_table, database::CalibrationDatabase& db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    // ERROR(Jack): Refactor the applications to pass a real recording name and hash here!
    RecordingId const recording_id{db.GetOrCreateRecording("todo_recording_name", "todo_recording_hash")};

    // TODO(Jack): We should refactor our config file so that it can load multiple cameras from one file and
    // automatically assign the index from the config file structure instead of harcoding it to 0 here. Same for the
    // targets and imus below.
    // TODO(Jack): Refactor so user can pass in a target name if they like.
    AssetId const camera_id{db.GetOrCreateAsset(AssetType::Camera, 0, cfg.camera.sensor_name)};
    AssetId const target_id{db.GetOrCreateAsset(AssetType::Target, 0, "aprilgrid")};

    std::optional<AssetId> imu_id;
    if (cfg.imu) {
        imu_id = db.GetOrCreateAsset(AssetType::Imu, 0, Name{cfg.imu->sensor_name});
    }

    log->info("{{'recording_id': '{}', 'assets': {{'camera_id': {}, 'target_id': {}, 'imu_id': {}}}}}",
              recording_id.value, camera_id.value, target_id.value, imu_id ? imu_id->value : -1);

    return {cfg, recording_id, camera_id, target_id, imu_id};
}

}  // namespace reprojection::steps
