#pragma once

#include "optimization/types.hpp"
#include "transforms/rig_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/optimization_types.hpp"

namespace reprojection::optimization {

struct BundleAdjustment {
    struct Problem {
        Problem(AssetId const rig_frame_asset_id, Frames const& rig_poses, std::map<AssetId, Camera> const& _cameras,
                std::vector<Observation> const& _observations)
            : rig{rig_frame_asset_id, rig_poses}, cameras{_cameras}, observations{_observations} {}

        Problem(AssetId const rig_frame_asset_id, Frames const& rig_poses) : rig{rig_frame_asset_id, rig_poses} {}

        // Update a problem with the optimized parts.
        Problem(Problem const& problem, Frames const& rig_poses, std::map<AssetId, CameraState> const& camera_states)
            : rig{problem.rig.asset_id, rig_poses}, cameras{problem.cameras}, observations{problem.observations} {
            for (auto& [camera_id, camera] : cameras) {
                camera.state = camera_states.at(camera_id);
            }
        }

        // State/parameterization
        DiscreteRig rig;
        std::map<AssetId, Camera> cameras;

        // Measurements
        std::vector<Observation> observations;
    };

    struct Result {
        explicit Result(Problem const& problem) : rig{problem.rig} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }

        Result(AssetId const& rig_frame_asset_id, Frames const& rig_poses,
               std::map<AssetId, CameraState> const& _camera_states)
            : rig{rig_frame_asset_id, rig_poses}, camera_states{_camera_states} {}

        DiscreteRig rig;
        std::map<AssetId, CameraState> camera_states;
    };

    static std::pair<Result, CeresState> Solve(Problem const& problem, int num_threads);

    static Problem MultiCamProblem(AssetId const& cam0_id, Frames const& cam0_poses,
                                   std::vector<CameraProblemInput> const& cams, uint64_t approx_sync_delta_ns);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, Frames const& frames, bool optimize_intrinsic,
                                    AssetId camera_id);

    // Single frame override - used for pnp nonlinear refinement of the DLT estimate.
    static Problem SingleFrameProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Bundle const& bundle,
                                      Pose const& pose, bool optimize_intrinsic);

   private:
    static void AddCamera(CameraProblemInput const& cam, uint64_t approx_sync_delta_ns, Problem& problem);
};

// TODO(Jack): Does this function really belong here in this file? Or would it be better organized with more like minded
// functions?
transforms::RigState ToRigState(BundleAdjustment::Result const& result);

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& problem);

}  // namespace reprojection::optimization
