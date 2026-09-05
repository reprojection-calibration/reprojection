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
        // This is the original measurement timestamp which we need to keep so we can satisfy foreign key constraints.
        uint64_t sample_timestamp_ns;
        // TODO NAMING! This is the timestamp of the synchronized rig frame!
        uint64_t frame_timestamp_ns;
        Bundle value;
    };

    struct Problem {
        Problem(std::map<AssetId, Camera> const& _cameras, Frames const& _rig_poses,
                std::vector<Observation> const& _observations)
            : cameras{_cameras}, rig_poses{_rig_poses}, observations{_observations} {}

        // Update a problem with the optimized parts.
        Problem(Problem const& problem, Frames const& _rig_poses, std::map<AssetId, CameraState> const& camera_states)
            : cameras{problem.cameras}, rig_poses{_rig_poses}, observations{problem.observations} {
            for (auto& [camera_id, camera] : cameras) {
                camera.state = camera_states.at(camera_id);
            }
        }

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

    static Problem MultiCamProblem(std::vector<CameraProblemInput> const& cameras, Frames const& rig_poses,
                                   uint64_t max_sync_delta_ns);

    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, Frames const& frames, bool optimize_intrinsic,
                                    AssetId camera_id);

    // Single frame override - used for pnp nonlinear refinement of the DLT estimate.
    static Problem SingleFrameProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic, Bundle const& bundle,
                                      Pose const& pose, bool optimize_intrinsic);

   private:
    static void AddCamera(CameraProblemInput const& data, uint64_t max_sync_delta_ns, bool optimize_intrinsic,
                          bool optimize_extrinsic, Problem& problem);
};

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& problem);

}  // namespace  reprojection::optimization
