#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::steps {

struct ExtrinsicInit {
    ExtrinsicInit(AssetId camera_id, StepId spline_id, AssetId imu_id, StepId imu_data_id, int num_threads,
                  SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    // NOTE(Jack): To actually just initialize the cam-imu extrinsics we actually only need the orientation component of
    // the spline. Assuming we decide to calculate reprojection errors in another step we need to refactor this!
    // TODO(Jack): Are there any problems with having this be a unique pointer? Is there any reason not just to give
    // spline a default constructor?
    std::unique_ptr<spline::Se3Spline> spline_;
    AssetId imu_id_;
    ImuMeasurements imu_data_;
    int num_threads_;
};

}  // namespace reprojection::steps
