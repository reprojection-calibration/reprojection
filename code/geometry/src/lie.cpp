#include "lie.hpp"

namespace reprojection::geometry {

Eigen::Matrix3d Exp(Eigen::Vector3d const& so3) {
    Eigen::Matrix3d const SO3{Eigen::AngleAxisd(so3.norm(), so3.normalized()).toRotationMatrix()};

    return SO3;
}

Eigen::Vector3d Log(Eigen::Matrix3d const& SO3) {
    Eigen::AngleAxisd const rotation(SO3);
    Eigen::Vector3d const so3{rotation.angle() * rotation.axis()};

    return so3;
}

}  // namespace reprojection::geometry