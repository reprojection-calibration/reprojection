#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct VisualInertialInit {
    VisualInertialInit(AssetId imu_id, StepId imu_data_id, AssetId cam_id, StepId spline_id, int num_threads,
                       SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    std::vector<AssetId> Assets() const { return {cam_id_, imu_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId imu_id_;
    StepId imu_data_id_;
    // NOTE(Jack): We only need to store the imu data so we can use it when we save the imu error diagnostics. It is not
    // actually used for any functional purpose besides that.
    ImuSamples imu_data_;
    AssetId cam_id_;
    // NOTE(Jack): To actually just initialize the cam-imu extrinsics we actually only need the orientation component of
    // the spline, but its easier to construct the entire spline as this fits better with out existing code semantics.
    // TODO(Jack): Are there any problems with having this be a unique pointer? Is there any reason not just to give
    // spline a default constructor?
    std::unique_ptr<spline::Se3Spline> spline_;

    int num_threads_;
};

}  // namespace reprojection::steps
