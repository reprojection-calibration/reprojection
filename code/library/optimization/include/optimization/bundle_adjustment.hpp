#pragma once

#include "optimization/types.hpp"
#include "spline/se3_spline.hpp"
#include "transforms/rig_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization::bundle_adjustment {

struct Discrete {
    struct Problem {
        // TODO(Jack): Do we need all three constructors?
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

// TODO(Jack): This is not merely continious it is also visual intertial! We should factor out the inertial part!
struct Continuous {
    // TODO WE NEED TO REFACTOR THE CLASS LAYOUTS! PUTTING THIS ALL IN ONE BA CLASS DOES NOT MAKE SENSE!
    struct Problem {
        Problem(AssetId const _rig_frame_asset_id, spline::Se3Spline const& _rig_spline, Array6d const& _se3_imu_rig,
                Vector3d const& _gravity_w)
            : rig_frame_asset_id{_rig_frame_asset_id},
              rig_spline{_rig_spline},
              se3_imu_rig{_se3_imu_rig},
              gravity_w{_gravity_w} {}

        AssetId rig_frame_asset_id;
        spline::Se3Spline rig_spline;
        Array6d se3_imu_rig;
        Vector3d gravity_w;
        std::map<AssetId, Camera> cameras;
        std::vector<Observation> observations;
    };

    // TODO clearly the spline, imu_rig extrinsic, and gravity form a vi ba relevant type.
    struct Result {
        AssetId rig_frame_asset_id;
        spline::Se3Spline rig_spline;
        Array6d se3_imu_rig;
        Vector3d gravity_w;
        std::map<AssetId, CameraState> camera_states;

        explicit Result(Problem const& problem)
            : rig_frame_asset_id{problem.rig_frame_asset_id},
              rig_spline{problem.rig_spline},
              se3_imu_rig{problem.se3_imu_rig},
              gravity_w{problem.gravity_w} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }
    };

    static std::pair<Result, CeresState> Solve(Problem const& problem, int num_threads);

    static Problem MultiCamProblem(AssetId const& cam0_id, spline::Se3Spline const& rig_spline,
                                   Array6d const& se3_imu_rig, Vector3d const& gravity_w,
                                   std::vector<CameraProblemInput> const& cams);

    // TODO(Jack): Rename from cam to rig?
    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, spline::Se3Spline const& rig_spline,
                                    Array6d const& se3_imu_rig, Vector3d const& gravity_w, AssetId camera_id);

   private:
    static void AddCamera(CameraProblemInput const& cam, Problem& problem);
};

// TODO(Jack): Does this function really belong here in this file? Or would it be better organized with more like minded
// functions?
transforms::RigState ToRigState(Discrete::Result const& result);

std::vector<ReprojectionError> EvaluateResiduals(Discrete::Problem const& problem);

}  // namespace reprojection::optimization::bundle_adjustment
