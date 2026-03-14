#pragma once

#include "projection_functions/camera_model.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Add all other camera models and check the above listed TODO points.
// TODO(Jack): Test!
std::unique_ptr<Camera> InitializeCamera(CameraModel const model, ArrayXd const& intrinsics, ImageBounds const& bounds);

template <typename T_Model>
    requires ProjectionClass<T_Model>
std::unique_ptr<Camera> MakeCamera(Eigen::VectorXd const& intrinsics, ImageBounds const& bounds) {
    if (intrinsics.rows() != T_Model::Size) {
        throw std::runtime_error("Intrinsic size mismatch - wanted " +          // LCOV_EXCL_LINE
                                 std::to_string(T_Model::Size) + " but got " +  // LCOV_EXCL_LINE
                                 std::to_string(intrinsics.rows()));            // LCOV_EXCL_LINE
    }

    using CameraType = Camera_T<T_Model>;

    return std::make_unique<CameraType>(intrinsics, bounds);
}

}  // namespace reprojection::projection_functions
