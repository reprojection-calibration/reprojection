#pragma once

#include "transforms/rig_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This is basically the data format that we need to do a bundle adjustment. As you can see it is highly
// duplicated from the native bundle adjustment types and therefore I think we have some representation unification
// coming in our future.
struct CameraProblemInput {
    AssetId camera_id;
    CameraInfo camera_info;
    Intrinsic intrinsic;
    TargetSamples targets;

    Array6d extrinsic{Array6d::Zero()};

    bool optimize_intrinsic{false};
    bool optimize_extrinsic{false};
};

struct BundleAdjustment {
    struct CameraState {
        Intrinsic intrinsic;
        // TODO(Jack): Frame order convention! Should we use the extrinsic type here? One reason that we do not is that
        // all the bundle adjustment extrinsics are with respect the reference rig asset. Therefore having the extrinsic
        // type here would almost be duplicating information. This is not clear yet.
        Array6d extrinsic;
    };

    struct CameraOptions {
        bool optimize_intrinsic{false};
        bool optimize_extrinsic{false};
    };

    struct Camera {
        CameraInfo camera_info;
        CameraState state;
        CameraOptions options;
    };

    struct Observation {
        AssetId camera_id;
        // This is the original measurement timestamp which we need to keep so we can satisfy foreign key constraints.
        uint64_t sample_timestamp_ns;
        // This is the timestamp of the synchronized rig frame!
        uint64_t frame_timestamp_ns;
        Bundle value;
    };

    struct Problem {
        // TODO DO WE STILL NEED THIS?
        Problem(AssetId const _rig_frame_asset_id, Frames const& _rig_poses, std::map<AssetId, Camera> const& _cameras,
                std::vector<Observation> const& _observations)
            : rig_frame_asset_id{_rig_frame_asset_id},
              rig_poses{_rig_poses},
              cameras{_cameras},
              observations{_observations} {}

        Problem(AssetId const _rig_frame_asset_id, Frames const& _rig_poses)
            : rig_frame_asset_id{_rig_frame_asset_id}, rig_poses{_rig_poses} {}

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
            : rig_frame_asset_id{problem.rig_frame_asset_id}, rig_poses{problem.rig_poses} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }

        Result(AssetId const& _rig_frame_asset_id, Frames const& _rig_poses,
               std::map<AssetId, CameraState> const& _camera_states)
            : rig_frame_asset_id{_rig_frame_asset_id}, rig_poses{_rig_poses}, camera_states{_camera_states} {}

        AssetId rig_frame_asset_id;
        Frames rig_poses;
        std::map<AssetId, CameraState> camera_states;
    };

    static std::pair<Result, CeresState> Solve(Problem const& ba_problem, int num_threads);

    static Problem MultiCamProblem(CameraProblemInput const& cam0, Frames const& cam0_poses,
                                   std::span<CameraProblemInput const> cams, uint64_t approx_sync_delta_ns);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, Frames const& frames, bool optimize_intrinsic,
                                    AssetId camera_id);

    // Single frame override - used for pnp nonlinear refinement of the DLT estimate.
    static Problem SingleFrameProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Bundle const& bundle,
                                      Pose const& pose, bool optimize_intrinsic);

   private:
    static void AddCamera(CameraProblemInput const& camera, uint64_t approx_sync_delta_ns, Problem& problem);
};

// TODO(Jack): Does this function really belong here in this file? Or would it be better organized with more like minded
// functions?
transforms::RigState ToRigState(BundleAdjustment::Result const& result);

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& problem);

}  // namespace  reprojection::optimization
