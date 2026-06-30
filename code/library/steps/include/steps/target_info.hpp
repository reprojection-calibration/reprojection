#pragma once

#include <toml++/toml.hpp>

#include "config/config_loading.hpp"
#include "config/config_parse.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct TargetInfoStep {
    config::Config::Target cfg_;
    // TODO(Jack): Is this really associated with a sensor or just an entity?
    std::string sensor_name_;

    static CalibrationStep StepType() { return CalibrationStep::TargetInfo; };

    std::string EntityId() const { return sensor_name_; }

    std::string HashInputs() const;

    TargetInfo Compute() const;

    TargetInfo Load(SqlitePtr const db) const;

    void Save(TargetInfo const& target_info, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
