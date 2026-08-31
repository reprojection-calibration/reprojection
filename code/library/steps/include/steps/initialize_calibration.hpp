#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// TODO(Jack): The core purpose of this struct is very similar to the config class itself, but we need some way of
// associating the parsed config with the database asset ids. If there is a nicer way to do this without than without
// essentially having two of the same structs and wrapping one with the Asset<> template struct lets do it.
struct CalibrationAssets {
    // TODO(Jack): Is this really the base name for what this does?
    template <typename T>
    struct Asset {
        AssetId id;
        T config;
    };

    std::vector<Asset<config::Config::Camera>> cameras;
    Asset<config::Config::Target> target;
    std::optional<Asset<config::Config::Imu>> imu;

    std::vector<AssetId> All() const {
        std::vector<AssetId> all_assets;

        for (auto const& [id, _] : cameras) {
            all_assets.push_back(id);
        }
        all_assets.push_back(target.id);
        if (imu) {
            all_assets.push_back(imu->id);
        }

        return all_assets;
    }
};

struct CalibrationContext {
    config::Config::Application application;
    CalibrationAssets assets;
    WorkflowId workflow_id;
    WorkflowType workflow_type;
};

CalibrationContext InitializeCalibration(toml::table const& cfg_table, SqlitePtr db);

WorkflowType DetermineWorkflowType(config::Config const& cfg);

CalibrationAssets CreateCalibrationAssets(config::Config const& cfg, SqlitePtr const db);

// TODO(Jack): Is there a simple way to test this without some huge database setup and implicit behavior checking?
void InsertAssetGroups(WorkflowType const workflow_type, CalibrationAssets const& assets, SqlitePtr const db);

}  // namespace reprojection::steps
