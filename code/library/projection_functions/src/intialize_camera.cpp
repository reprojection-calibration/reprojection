

#include "projection_functions/camera_model.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Add all other camera models and check the above listed TODO points.
std::unique_ptr<Camera> InitializeCamera(CameraModel const model, ArrayXd const& intrinsics,
                                         ImageBounds const& bounds) {
    if (model == CameraModel::DoubleSphere) {
        if (not(intrinsics.rows() == DoubleSphere::Size)) {
            throw std::runtime_error(
                "InitializeCamera() requested CameraModel::DoubleSphere but got intrinsics with length: " +  // LCOV_EXCL_LINE
                std::to_string(intrinsics.rows()));  // LCOV_EXCL_LINE
        }
        return std::unique_ptr<Camera>(new DoubleSphereCamera(intrinsics, bounds));
    } else if (model == CameraModel::Pinhole) {
        if (not(intrinsics.rows() == Pinhole::Size)) {
            throw std::runtime_error(
                "InitializeCamera() requested CameraModel::Pinhole but got intrinsics with length: " +  // LCOV_EXCL_LINE
                std::to_string(intrinsics.rows()));  // LCOV_EXCL_LINE
        }
        return std::unique_ptr<Camera>(new PinholeCamera(intrinsics, bounds));
    } else if (model == CameraModel::PinholeRadtan4) {
        if (not(intrinsics.rows() == PinholeRadtan4::Size)) {
            throw std::runtime_error(
                "InitializeCamera() requested CameraModel::PinholeRadtan4 but got intrinsics with length: " +  // LCOV_EXCL_LINE
                std::to_string(intrinsics.rows()));  // LCOV_EXCL_LINE
        }
        return std::unique_ptr<Camera>(new PinholeRadtan4Camera(intrinsics, bounds));
    } else if (model == CameraModel::UnifiedCameraModel) {
        if (not(intrinsics.rows() == UnifiedCameraModel::Size)) {
            throw std::runtime_error(
                "InitializeCamera() requested CameraModel::UnifiedCameraModel but got intrinsics with length: " +  // LCOV_EXCL_LINE
                std::to_string(intrinsics.rows()));  // LCOV_EXCL_LINE
        }
        return std::unique_ptr<Camera>(new UcmCamera(intrinsics, bounds));
    } else {
        throw std::runtime_error("invalid camera model");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::projection_functions