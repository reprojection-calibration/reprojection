#pragma once

#include "types/transform_types.hpp"

// TODO(Jack): Does this need to be a public header?

namespace reprojection::transforms {

class Extrinsics {
   public:
    explicit Extrinsics(std::vector<Extrinsic> const& values);

    std::vector<Extrinsic> Values() const { return values_; }

    bool HasPath(AssetId frame_a, AssetId frame_b) const;

    Array6d Resolve(AssetId frame_a, AssetId frame_b) const;

    // NOTE(Jack): This is the real core graph/path search algorithm below here. We made FindPath() static so we could
    // easily test it.
    struct PathEdge {
        Extrinsic extrinsic;
        // true: b -> a
        // false: a -> b
        bool forward;
    };
    using Path = std::vector<PathEdge>;

    static std::optional<Path> FindPath(std::vector<Extrinsic> const& values, AssetId frame_a, AssetId frame_b);

   private:
    std::vector<Extrinsic> values_;
};

}  // namespace reprojection::transforms