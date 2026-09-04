#pragma once

#include <tuple>

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

struct BundleAdjustment {
    struct CameraStateXxx {
        Intrinsic intrinsic;
        Array6d se3_co_rig;
    };

    struct CameraOptions {
        bool optimize_intrinsic{true};
        bool optimize_extrinsic{true};
    };

    struct Camera {
        CameraInfo camera_info;
        CameraStateXxx state;
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
        std::map<AssetId, CameraStateXxx> camera_states;
    };
};

}  // namespace  reprojection::optimization
