#include "pose_utilities.hpp"

namespace reprojection::pnp {

Se3 ToSe3(Eigen::Isometry3d const& matrix) {
    Eigen::AngleAxisd const rotation(matrix.linear());
    Eigen::Vector3d const so3{rotation.angle() * rotation.axis()};

    Se3 pose;
    pose << so3, matrix.translation();

    return pose;
}

Eigen::Isometry3d FromSe3(Se3 const& se3) {
    Eigen::Isometry3d pose;

    Eigen::Vector3d const so3{se3.topRows(3)};
    pose.linear() = Eigen::AngleAxisd(so3.norm(), so3.normalized()).toRotationMatrix();
    pose.translation() = se3.bottomRows(3);

    return pose;
}

}  // namespace reprojection::pnp