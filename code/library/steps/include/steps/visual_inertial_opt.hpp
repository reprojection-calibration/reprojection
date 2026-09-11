#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"
#include "types/transform_types.hpp"

namespace reprojection::steps {

// VisualInertialInit(AssetId imu_id, StepId imu_data_id, AssetId cam_id, StepId spline_id, int num_threads, SqlitePtr
// db);

struct VisualInertialOpt {
    // TODO(Jack): Can we refactor this to take a CamStageIds struct to shorten this up a little?
    VisualInertialOpt(AssetId imu_id, StepId imu_data_id, AssetId cam_id, StepId spline_id, StepId extrinsic_init_id,
                      StepId targets_id, StepId camera_info_id, StepId intrinsic_id, int num_threads, SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicOptimization; }

    std::vector<AssetId> Assets() const { return {imu_id_, cam_id_}; }  // LCOV_EXCL_LINE

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId imu_id_;
    StepId imu_data_id_;
    ImuSamples imu_data_;
    AssetId cam_id_;
    std::unique_ptr<spline::Se3Spline> spline_;
    Extrinsic extrinsic_;
    Vector3d gravity_;
    StepId targets_id_;
    TargetSamples targets_;
    CameraInfo camera_info_;
    Intrinsic intrinsic_;

    int num_threads_;
};

}  // namespace reprojection::steps
