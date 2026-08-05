#pragma once

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct TargetInfoStep {
    TargetInfoStep(AssetId target_id, config::Config::Target const& target);

    static StepType Type() { return StepType::TargetInfo; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase const& db) const;

   private:
    AssetId target_id_;
    config::Config::Target target_;
};

}  // namespace reprojection::steps
