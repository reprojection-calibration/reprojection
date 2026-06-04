#pragma once

#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct DoNotUse {};

struct SplineReprojectionError {
    std::string SensorName() const { return "TODO!!!"; }

    std::string CacheKey() const;

    DoNotUse Compute() const;

    DoNotUse Load(SqlitePtr const db) const;

    void Save(DoNotUse const do_not_use, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
