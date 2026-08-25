#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct TargetInfoStep {
    TargetInfoStep(AssetId target_id, config::Config::Target const& target);

    static StepType Type() { return StepType::TargetInfo; }

    std::vector<AssetId> Assets() const { return {target_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId target_id_;
    config::Config::Target target_;
};

}  // namespace reprojection::steps
