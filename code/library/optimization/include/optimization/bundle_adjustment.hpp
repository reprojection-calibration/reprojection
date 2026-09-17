#pragma once

#include "optimization/types.hpp"
#include "spline/se3_spline.hpp"
#include "transforms/rig_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/optimization_types.hpp"

namespace reprojection::optimization {

struct Discrete {
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

struct VisualInertial {
    struct Problem {
        Problem(AssetId const rig_frame_asset_id, spline::Se3Spline const& rig_spline, Array6d const& se3_imu_rig,
                Vector3d const& gravity_w, ImuSamples const& _imu_data)
            : rig{rig_frame_asset_id, rig_spline}, inertial_state{se3_imu_rig, gravity_w}, imu_data{_imu_data} {}

        // Update a problem with the optimized parts.
        // TODO(Jack): I would like to use the Result type here as the parameter to pass but we need forward declaration
        // and split definitions. It got messy so I decided just to pass the args directly for now.
        Problem(Problem const& problem, ContinuousRig const& _rig, InertialState const& _inertial_state,
                std::map<AssetId, CameraState> const& camera_states)
            : rig{_rig},
              inertial_state{_inertial_state},
              cameras{problem.cameras},
              observations{problem.observations},
              imu_data{problem.imu_data} {
            for (auto& [camera_id, camera] : cameras) {
                camera.state = camera_states.at(camera_id);
            }
        }

        // State/parameterization
        ContinuousRig rig;
        InertialState inertial_state;
        std::map<AssetId, Camera> cameras;

        // Measurements
        std::vector<Observation> observations;
        ImuSamples imu_data;
    };

    struct Result {
        ContinuousRig rig;
        InertialState inertial_state;
        std::map<AssetId, CameraState> camera_states;

        explicit Result(Problem const& problem) : rig{problem.rig}, inertial_state{problem.inertial_state} {
            for (auto const& [camera_id, camera] : problem.cameras) {
                camera_states.emplace(camera_id, camera.state);
            }
        }
    };

    static std::pair<Result, CeresState> Solve(Problem const& problem, int num_threads);

    static Problem MultiCamProblem(AssetId const& cam0_id, spline::Se3Spline const& rig_spline,
                                   Array6d const& se3_imu_rig, Vector3d const& gravity_w,
                                   std::vector<CameraProblemInput> const& cams, ImuSamples const& imu_data);

    // TODO(Jack): Rename from cam to rig?
    // TODO(Jack): I think this is only used for testing! Can we refactor this away?
    static Problem SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                    TargetSamples const& targets, spline::Se3Spline const& rig_spline,
                                    Array6d const& se3_imu_rig, Vector3d const& gravity_w, AssetId camera_id,
                                    ImuSamples const& imu_data);

   private:
    static void AddCamera(CameraProblemInput const& cam, Problem& problem);
};

// TODO(Jack): Does this function really belong here in this file? Or would it be better organized with more like minded
// functions?
transforms::RigState ToRigState(Discrete::Result const& result);

std::vector<ReprojectionError> EvaluateResiduals(Discrete::Problem const& problem);

}  // namespace reprojection::optimization
