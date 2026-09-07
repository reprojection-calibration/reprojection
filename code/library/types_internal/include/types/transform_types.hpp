#pragma once

#include <map>

#include "types/database_types.hpp"
#include "types/eigen_types.hpp"

#include "calibration_types.hpp"

namespace reprojection {

struct Extrinsic {
    AssetId frame_a;
    AssetId frame_b;
    Array6d se3_a_b;
};

// TODO(Jack): Is this the right file for this type?
struct RigState {
    // NOTE(Jack): As of now we have the logical constraint that the rig must always be referenced to one of the
    // existing sensors (really only the cameras) because we have no other way ot initializing the rigs poses and
    // camera-rig extrinsic. If one day we had a way to do this (i.e. with mocap and known initial extrinsics) then
    // maybe we need to add a custom asset just for the rig reference frame, but we are not there yet!
    AssetId rig_frame_asset_id;
    // NOTE(Jack): These poses are of the rig_frame asset in the world frame. So in some sense the rig_frame_asset_id
    // and poses are uniquely connected/related.
    Frames poses;
    // NOTE(Jack): The extrinsic for the rig_frame_asset_id should always be identity! At least until one day we get
    // maybe more complicated non-camera centric initialization methods. But that is really future music.
    std::map<AssetId, Array6d> extrinsics;
};

}  // namespace reprojection