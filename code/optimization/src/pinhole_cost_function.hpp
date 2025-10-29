#pragma once

#include "ceres_geometry.hpp"
#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
// TODO(Jack): Do we need to template the entire class? Or can we do just the subfunctions?
// WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
// referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
// that but consider how we  can design the program to handle that automatically.
// NOTE(Jack): The operator() method  is the contact line were ceres requirements for using raw pointers hits
// our desire to use more expressive eigen types. That is why here in the operator() we find the usage of the Eigen::Map
// class.
template <typename T_Model, int NumIntrinsics>
class ProjectionCostFunction_T {
   public:
    explicit ProjectionCostFunction_T(Vector2d const& pixel, Vector3d const& point) : pixel_{pixel}, point_{point} {}

    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const pose_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 6> const> pose(pose_ptr);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Map<Eigen::Array<T, NumIntrinsics, 1> const> intrinsics(intrinsics_ptr);
        Eigen::Vector<T, 2> const pixel{T_Model::template Project<T>(intrinsics, point_co)};

        residual[0] = T(pixel_[0]) - pixel[0];
        residual[1] = T(pixel_[1]) - pixel[1];

        return true;
    }

    Vector2d pixel_;
    Vector3d point_;
};

// TODO(Jack): We need to associate the size of the intriscs directly with the model. By passing both in here we are
// really duplicating information.
template <typename T_Model, int NumIntrinsics>
ceres::CostFunction* Create_T(Vector2d const& pixel, Vector3d const& point) {
    using ProjectionCostFunction = ProjectionCostFunction_T<T_Model, NumIntrinsics>;

    return new ceres::AutoDiffCostFunction<ProjectionCostFunction, 2, NumIntrinsics, 6>(
        new ProjectionCostFunction(pixel, point));
}

// TODO(Jack): Does the intrinsic size actually belong/should be taken from the static camera class?
enum class CameraModel {
    DoubleSphere = 6,  //
    Pinhole = 4,
    PinholeRadtan4 = 8,
    UnifiedCameraModel = 5
};

ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point) {
    if (projection_type == CameraModel::DoubleSphere) {
        return Create_T<projection_functions::DoubleSphere, static_cast<int>(CameraModel::DoubleSphere)>(pixel, point);
    } else if (projection_type == CameraModel::Pinhole) {
        return Create_T<projection_functions::Pinhole, static_cast<int>(CameraModel::Pinhole)>(pixel, point);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return Create_T<projection_functions::PinholeRadtan4, static_cast<int>(CameraModel::PinholeRadtan4)>(pixel,
                                                                                                             point);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return Create_T<projection_functions::UnifiedCameraModel, static_cast<int>(CameraModel::UnifiedCameraModel)>(
            pixel, point);
    } else {
        throw std::runtime_error("BLAH BLAH BLAH");
    }
}

}  // namespace  reprojection::optimization
