#include "steps/initialize_calibration.hpp"


namespace reprojection::steps {

CalibrationContext InitializeCalibration(toml::table const& cfg_table, database::CalibrationDatabase& db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    // TODO(Jack): Refactor the applications to pass a real recording name and hash here!
    RecordingId const recording_id{db.GetOrCreateRecording("todo_recording_name", "todo_recording_hash")};
    // TODO(Jack): By passing the entire config here that means we will hash even non-calibration related things like if
    // the feature extraction display. We should instead I think design a hashing function for the actual config::Config
    // struct and pass that/use that here.
    // ERROR(Jack): Pass the actual config here!
    RunId const run_id{db.GetOrCreateRun(recording_id, "")};

    // TODO(Jack): We should refactor our config file so that it can load multiple cameras from one file and
    // automatically assign the index from the config file structure instead of harcoding it to 0 here. Same for the
    // targets and imus below.
    AssetId const camera_id{db.GetOrCreateAsset(AssetType::Camera, 0, cfg.camera.sensor_name)};
    AssetId const target_id{db.GetOrCreateAsset(AssetType::Target, 0, Name{"calibration_target"})};

    std::optional<AssetId> imu_id;
    if (cfg.imu) {
        imu_id = db.GetOrCreateAsset(AssetType::Imu, 0, Name{cfg.imu->sensor_name});
    }

    return {cfg, recording_id, run_id, camera_id, target_id, imu_id};
}

}  // namespace reprojection::steps
