#pragma once

#include "types/transform_types.hpp"

namespace reprojection::transforms {

class Extrinsics {
   public:
    explicit Extrinsics(std::vector<Extrinsic> const& values);

    std::vector<Extrinsic> Values() const { return values_; }

    bool HasPath(AssetId frame_a, AssetId frame_b) const;

    Array6d Resolve(AssetId frame_a, AssetId frame_b) const;

    // NOTE(Jack): Below this point is the real core "path finding" algorithm logic which drives the actually useful
    // class interface above. We made FindPath() static so it was easily testable.
    struct PathEdge {
        Extrinsic extrinsic;
        bool forward;
    };

    using Path = std::vector<PathEdge>;

    static std::optional<Path> FindPath(std::vector<Extrinsic> const& values, AssetId frame_a, AssetId frame_b);

   private:
    std::vector<Extrinsic> values_;
};

}  // namespace reprojection::transforms