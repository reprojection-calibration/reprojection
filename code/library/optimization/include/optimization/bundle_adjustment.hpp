#pragma once

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

struct BundleAdjustment {
    struct CameraState {
        Intrinsic intrinsic;
        // TODO(Jack): Frame order convention! Should we use the extrinsic type here?
        Array6d extrinsic;
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
        Bundle value;
    };

    struct Problem {
        std::map<AssetId, Camera> cameras;
        Frames rig_poses;
        std::vector<Observation> observations;
    };

    struct Result {
        explicit Result(Problem const& problem)
            : rig_poses{problem.rig_poses}, ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }

        Frames rig_poses;
        CeresState ceres_state;
        std::map<AssetId, CameraState> camera_states;
    };

    static Result Solve(Problem const& ba_problem, int num_threads);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Frames const& frames,
                                    TargetSamples const& targets, bool optimize_intrinsic,
                                    AssetId camera_id = AssetId{0});

    // Single frame override - used for pnp nonlinear refinement of the DLT estimate.
    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Pose const& pose,
                                    Bundle const& bundle, bool optimize_intrinsic);
};

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& problem);

}  // namespace  reprojection::optimization
