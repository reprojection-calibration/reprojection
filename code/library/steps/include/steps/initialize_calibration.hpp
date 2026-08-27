#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct CalibrationAssets {
    std::vector<AssetId> cameras;
    AssetId target;
    std::optional<AssetId> imu;

    std::vector<AssetId> All() const {
        std::vector<AssetId> all_assets{cameras};
        all_assets.push_back(target);

        if (imu) {
            all_assets.push_back(*imu);
        }

        return all_assets;
    }
};

struct CalibrationContext {
    config::Config config;
    // TODO STORE ID OR TYPE HERE OR BOTH?
    WorkflowId workflow_id;
    CalibrationAssets assets;
};

CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr db);

WorkflowType DetermineWorkflowType(config::Config const& cfg);

CalibrationAssets CreateCalibrationAssets(config::Config const& cfg, SqlitePtr const db);

}  // namespace reprojection::steps
