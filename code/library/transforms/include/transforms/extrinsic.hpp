#pragma once

#include "types/transform_types.hpp"

// TODO(Jack): Does this need to be a public header?

namespace reprojection::transforms {

class Extrinsics {
   public:
    explicit Extrinsics(std::vector<Extrinsic> const& values);

    bool HasPath(AssetId frame_a, AssetId frame_b) const;

    Array6d Resolve(AssetId frame_a, AssetId frame_b) const;

   private:
    std::vector<Extrinsic> values_;
};

}  // namespace reprojection::transforms