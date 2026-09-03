#pragma once

#include <tuple>

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

struct BaCameraState {
    CameraState intrinsic;
    Array6d se3_co_rig;
};

struct BaCameraOptions {
    bool optimize_intrinsic{true};
    bool optimize_extrinsic{true};
};

struct BaCamera {
    CameraInfo camera_info;
    BaCameraState state;
    BaCameraOptions options;
};

struct BaObservation {
    AssetId camera_id;
    uint64_t timestamp_ns;
    Bundle bundle;
};

struct BaProblem {
    std::map<AssetId, BaCamera> cameras;
    Frames frames;
    std::vector<BaObservation> observations;
};

struct BaResult {
    explicit BaResult(BaProblem const& problem)
        : frames{problem.frames}, ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR} {
        for (auto const& [camera_id, camera] : problem.cameras) {
            camera_states.emplace(camera_id, camera.state);
        }
    }

    Frames frames;
    CeresState ceres_state;
    std::map<AssetId, BaCameraState> camera_states;
};

// TODO(Jack): This function has scarily many parameters! Is this a problem or sign of a bad design?
BaResult BundleAdjustment(BaProblem const& ba_problem, int const num_threads);

ReprojectionErrors ReprojectionError(CameraInfo const& sensor, CameraMeasurements const& targets,
                                     OptimizationState const& state);

BaProblem BuildSingleCamBaProblem(CameraInfo const& camera_info, CameraState const& intrinsics, Frames const& frames,
                                  CameraMeasurements const& targets, bool const optimize_intrinsic = true);

}  // namespace  reprojection::optimization
