#include "geometry/lie.hpp"

namespace reprojection::geometry {

Eigen::Isometry3d Exp(Eigen::Vector<double, 6> const& se3) {
    Eigen::Isometry3d SE3;

    Eigen::Vector3d const so3{se3.topRows(3)};
    SE3.linear() = Exp(so3);
    SE3.translation() = se3.bottomRows(3);

    return SE3;
}

Eigen::Vector<double, 6> Log(Eigen::Isometry3d const& SE3) {
    Eigen::Vector3d const so3{Log(SE3.linear())};

    Eigen::Vector<double, 6> pose;
    pose << so3, SE3.translation();

    return pose;
}

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