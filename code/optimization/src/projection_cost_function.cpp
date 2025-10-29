#include "projection_cost_function.hpp"

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point) {
    if (projection_type == CameraModel::DoubleSphere) {
        return Create_T<projection_functions::DoubleSphere>(pixel, point);
    } else if (projection_type == CameraModel::Pinhole) {
        return Create_T<projection_functions::Pinhole>(pixel, point);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return Create_T<projection_functions::PinholeRadtan4>(pixel, point);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return Create_T<projection_functions::UnifiedCameraModel>(pixel, point);
    } else {
        // TODO(Jack): Actually assess and handle possible error conditions!
        throw std::runtime_error("BLAH BLAH BLAH");
    }
}

}  // namespace  reprojection::optimization
