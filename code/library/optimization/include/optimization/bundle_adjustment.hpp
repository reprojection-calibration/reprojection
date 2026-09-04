#pragma once

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

struct BundleAdjustment {
    struct CameraState {
        Intrinsic intrinsic;
        Array6d se3_co_rig;
    };

    struct CameraOptions {
        bool optimize_intrinsic{true};
        bool optimize_extrinsic{true};
    };

    struct Camera {
        CameraInfo camera_info;
        CameraState state;
        CameraOptions options;
    };

    struct Observation {
        AssetId camera_id;
        uint64_t timestamp_ns;
        Bundle bundle;
    };

    struct Problem {
        std::map<AssetId, Camera> cameras;
        Frames frames;
        std::vector<Observation> observations;
    };

    struct Result {
        explicit Result(Problem const& problem)
            : frames{problem.frames}, ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }

        Frames frames;
        CeresState ceres_state;
        std::map<AssetId, CameraState> camera_states;
    };

    static Result Solve(Problem const& ba_problem, int num_threads);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsics, Frames const& frames,
                                    TargetSamples const& targets, bool optimize_intrinsic = true);
};

ReprojectionErrors ReprojectionError(CameraInfo const& sensor, TargetSamples const& targets,
                                     OptimizationState const& state);

}  // namespace  reprojection::optimization
