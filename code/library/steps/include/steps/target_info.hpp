#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct TargetInfoStep {
    toml::table target_config;
    std::string sensor_name;

    CalibrationStep step_type{CalibrationStep::TargetInfo};

    std::string EntityId() const { return sensor_name; }

    std::string CacheKey() const;

    TargetInfo Compute() const;

    TargetInfo Load(SqlitePtr const db) const;

    void Save(TargetInfo const& target_info, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
