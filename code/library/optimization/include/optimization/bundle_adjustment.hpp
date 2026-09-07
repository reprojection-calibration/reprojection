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
        Problem(AssetId const _rig_frame_asset_id, Frames const& _rig_poses, std::map<AssetId, Camera> const& _cameras,
                std::vector<Observation> const& _observations)
            : rig_frame_asset_id{_rig_frame_asset_id},
              rig_poses{_rig_poses},
              cameras{_cameras},
              observations{_observations} {}

        // Update a problem with the optimized parts.
        Problem(Problem const& problem, Frames const& _rig_poses, std::map<AssetId, CameraState> const& camera_states)
            : rig_frame_asset_id{problem.rig_frame_asset_id},
              rig_poses{_rig_poses},
              cameras{problem.cameras},
              observations{problem.observations} {
            for (auto& [camera_id, camera] : cameras) {
                camera.state = camera_states.at(camera_id);
            }
        }

        AssetId rig_frame_asset_id;
        Frames rig_poses;
        std::map<AssetId, Camera> cameras;
        std::vector<Observation> observations;
    };

    struct Result {
        explicit Result(Problem const& problem)
            : rig_frame_asset_id{problem.rig_frame_asset_id},
              rig_poses{problem.rig_poses},
              ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }

        AssetId rig_frame_asset_id;
        Frames rig_poses;
        std::map<AssetId, CameraState> camera_states;
        CeresState ceres_state;
    };

    static Result Solve(Problem const& ba_problem, int num_threads);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, Frames const& frames, bool optimize_intrinsic,
                                    AssetId camera_id);

    // Single frame override - used for pnp nonlinear refinement of the DLT estimate.
    static Problem SingleFrameProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Bundle const& bundle,
                                      Pose const& pose, bool optimize_intrinsic);
};

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& problem);

}  // namespace  reprojection::optimization
