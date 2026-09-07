#include "transforms/extrinsic.hpp"

#include <format>
#include <set>

#include "geometry/lie.hpp"

namespace reprojection::transforms {

Extrinsics::Extrinsics(std::vector<Extrinsic> const& values) {
    // Check that the provided extrinsic do not make a invalid extrinsic tree (i.e. no cycles and no self transform).
    for (auto const& extrinsic : values) {
        if (extrinsic.frame_a == extrinsic.frame_b) {
            throw std::invalid_argument{
                std::format("No extrinsic path between frames! frame_a (asset_id {}) and frame_b (asset_id {})",
                            extrinsic.frame_a.value, extrinsic.frame_b.value)};
        } else if (FindPath(values_, extrinsic.frame_a, extrinsic.frame_b)) {
            throw std::invalid_argument{
                std::format("No extrinsic path between frames! frame_a (asset_id {}) and frame_b (asset_id {})",
                            extrinsic.frame_a.value, extrinsic.frame_b.value)};
        }

        // We incrementally "accept" the provided extrinsics so we can check that there are no cycles that get created
        // as we construct the graph.
        values_.push_back(extrinsic);
    }
}

bool Extrinsics::HasPath(AssetId const frame_a, AssetId const frame_b) const {
    return this->FindPath(values_, frame_a, frame_b).has_value();
}

Array6d Extrinsics::Resolve(AssetId const frame_a, AssetId const frame_b) const {
    auto const path{FindPath(values_, frame_a, frame_b)};
    if (not path) {
        throw std::runtime_error{
            std::format("No extrinsic path between frames! frame_a (asset_id {}) and frame_b (asset_id {})",
                        frame_a.value, frame_b.value)};
    }

    Isometry3d tf_a_b{Isometry3d::Identity()};
    for (auto const& [extrinsic, forward] : *path) {
        Isometry3d const tf_i{geometry::Exp(extrinsic.se3_a_b)};

        tf_a_b = (forward ? tf_i : tf_i.inverse()) * tf_a_b;
    }

    return geometry::Log(tf_a_b);
}

// NOTE(Jack): This function does not guarantee that there are no cycles in the extrinsics, that is the responsibility
// of the Extrinsics class constructor.
std::optional<Extrinsics::Path> Extrinsics::FindPath(std::vector<Extrinsic> const& values, AssetId const frame_a,
                                                     AssetId const frame_b) {
    // TODO(Jack): Does it make sense that even if the frames are not present inside of the intrinsic vector that it
    // still returns identity here? It almost seems like it makes more sense to throw and error if you ask for a tf
    // between two frames that are not even present in the extrinsic.
    if (frame_a == frame_b) {
        return Path{};
    }

    struct PendingFrame {
        AssetId frame;
        Path path;
    };
    std::vector<PendingFrame> pending_frames{{frame_b, {}}};
    std::set<AssetId> visited_frames;

    while (not std::empty(pending_frames)) {
        auto [current_frame, path] = std::move(pending_frames.back());
        pending_frames.pop_back();

        // Insert on a set returns false if the value is already present in the set.
        if (not visited_frames.insert(current_frame).second) {
            continue;
        }

        for (auto const& extrinsic : values) {
            AssetId next_frame;
            bool forward;

            if (extrinsic.frame_a == current_frame) {
                next_frame = extrinsic.frame_b;
                forward = false;
            } else if (extrinsic.frame_b == current_frame) {
                next_frame = extrinsic.frame_a;
                forward = true;
            } else {
                continue;
            }

            auto next_path{path};
            next_path.push_back({extrinsic, forward});

            if (next_frame == frame_a) {
                return next_path;
            }

            pending_frames.push_back({
                next_frame,
                std::move(next_path),
            });
        }
    }

    return std::nullopt;
}

}  // namespace reprojection::transforms
