#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// TODO(Jack): This is not actually a step itself so why is it in the steps module? I tried to put it in the application
// module but that interface is too public there for methods that are only used internally.

// TODO(Jack): The core purpose of this struct is very similar to the config class itself, but we need some way of
// associating the parsed config with the database asset ids. If there is a nicer way to do this without than without
// essentially having two of the same structs and wrapping one with the Asset<> template struct lets do it.
struct CalibrationAssets {
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
    }  // LCOV_EXCL_LINE
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

// NOTE(Jack): There are three basic types of asset groups. This should really be officially documented somewhere but we
// are not at that point yet. The only one you really need to actively maintain are the "custom subgroups" which can
// change depending on which calibration types are added or refactored.
//
//      1) Each asset is its own asset group (assigned to steps)
//      2) The entire set of assets is a asset group (assigned to the workflow and possible a step)
//      3) Custom subgroups can be assets groups (assigned to steps)
//
// This is done to express a sort of ownership semantic for the steps. We need to specify which assets are associated
// with process steps so we can uniquely specify them in the database. Because some steps are "owned" or involve more
// than just one asset we created the idea of asset groups, which can include one or more asset ids.
//
//
// TODO(Jack): Is there a simple way to test this without some huge database setup and implicit behavior checking?
void InsertAssetGroups(WorkflowType const workflow_type, CalibrationAssets const& assets, SqlitePtr const db);

}  // namespace reprojection::steps
