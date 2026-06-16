#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct TargetInfoStep {
    toml::table target_config_;
    std::string sensor_name_;

    CalibrationStep step_type{CalibrationStep::TargetInfo};

    std::string EntityId() const { return sensor_name_; }

    std::string HashInputs() const;

    TargetInfo Compute() const;

    TargetInfo Load(SqlitePtr const db) const;

    void Save(TargetInfo const& target_info, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
