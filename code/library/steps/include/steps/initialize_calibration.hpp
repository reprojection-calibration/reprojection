#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct CalibrationContext {
    config::Config config;

    WorkflowId workflow_id;

    AssetId camera_id;
    AssetId target_id;
    std::optional<AssetId> imu_id;
};

CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr db);

}  // namespace reprojection::steps
