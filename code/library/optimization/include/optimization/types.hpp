#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This is basically the data format that we need to do a bundle adjustment. As you can see it is highly
// duplicated from the native bundle adjustment types and therefore I think we have some representation unification
// coming in our future.
// TODO(Jack): Where does this type belong?
struct CameraProblemInput {
    AssetId camera_id;
    CameraInfo camera_info;
    Intrinsic intrinsic;
    TargetSamples targets;

    Array6d extrinsic{Array6d::Zero()};

    bool optimize_intrinsic{false};
    bool optimize_extrinsic{false};
};

}  // namespace  reprojection::optimization

namespace reprojection::optimization::bundle_adjustment {

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

}  // namespace reprojection::optimization::bundle_adjustment