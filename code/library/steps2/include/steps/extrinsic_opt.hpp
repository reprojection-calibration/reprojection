#pragma once

#include "database/calibration_database.hpp"


namespace reprojection::steps {

struct ExtrinsicOptimization {
    ExtrinsicOptimization(
                          database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::ExtrinsicOptimization; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:

};

}  // namespace reprojection::steps
