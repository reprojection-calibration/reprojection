#include "transforms/extrinsic.hpp"

#include <set>

namespace reprojection::transforms {

Extrinsics::Extrinsics(std::vector<Extrinsic> const& values) : values_{values} {}

bool Extrinsics::HasPath(AssetId const frame_a, AssetId const frame_b) const {
    // TODO(Jack): Is that weird that an asset to itself always works even if the assets are not even in the extrinsic
    // set? I would almost expect and error.
    if (frame_a == frame_b) {
        return true;
    }

    std::set<AssetId> visited_frames;
    std::vector pending_frames{frame_a};
    while (not std::empty(pending_frames)) {
        AssetId const current_frame{pending_frames.back()};
        pending_frames.pop_back();

        // NOTE(Jack): Here 'set.insert()' returns a pair with a boolean that tells you if the insertion was successful
        // (element not already in the set) or not (element already in the set). This is what prevents us from visiting
        // the same frame twice.
        if (not visited_frames.insert(current_frame).second) {
            continue;
        }

        for (auto const& extrinsic : values_) {
            if (extrinsic.frame_a == current_frame) {
                if (extrinsic.frame_b == frame_b) {
                    return true;
                } else {
                    pending_frames.push_back(extrinsic.frame_b);
                }
            } else if (extrinsic.frame_b == current_frame) {
                if (extrinsic.frame_a == frame_b) {
                    return true;
                } else {
                    pending_frames.push_back(extrinsic.frame_a);
                }
            }
        }
    }

    return false;
}

Array6d Extrinsics::Resolve(AssetId const frame_a, AssetId const frame_b) const {
    // TODO(Jack): See note in HasPath() about this maybe being an error.
    if (frame_a == frame_b) {
        return Array6d::Zero();
    }

    return Array6d::Zero();
}

}  // namespace reprojection::transforms
