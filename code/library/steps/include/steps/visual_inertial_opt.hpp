#pragma once

#include "optimization/types.hpp"
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
    VisualInertialOpt(AssetId imu_id, StepId imu_data_id, AssetId cam0_id, std::vector<CamStageIds> const& cams,
                      StepId spline_id, StepId extrinsic_init_id, int num_threads, SqlitePtr db);

    static StepType Type() { return StepType::VisualInertialOpt; }

    std::vector<AssetId> Assets() const {
        std::vector assets{imu_id_};
        for (auto const& camera : ba_input_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(camera.camera_id);
        }

        return assets;
    }  // LCOV_EXCL_LINE

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId imu_id_;
    StepId imu_data_id_;
    ImuSamples imu_data_;
    AssetId cam0_id_;
    std::unique_ptr<spline::Se3Spline> spline_;
    Extrinsic extrinsic_imu_rig_;
    Vector3d gravity_;
    // NOTE(Jack): We need to store these correspondences so we can satisfy the database foreign key constraints on
    // reprojection error. If this is a long term strategy time will tell!
    std::map<AssetId, StepId> cam_target_ids_;
    std::vector<optimization::CameraProblemInput> ba_input_;

    int num_threads_;
};

}  // namespace reprojection::steps
