#pragma once

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "types/database_types.hpp"

namespace reprojection::steps {

struct CalibrationContext {
    config::Config config;

    RecordingId recording_id;
    RunId run_id;

    AssetId camera_id;
    AssetId target_id;
    std::optional<AssetId> imu_id;
};

CalibrationContext InitializeCalibration(toml::table const& cfg_table, database::CalibrationDatabase& db);

}  // namespace reprojection::steps
