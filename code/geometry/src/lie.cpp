#include "geometry/lie.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3) {
    Isometry3d SE3;

    Vector3d const so3{se3.topRows(3)};
    SE3.linear() = Exp(so3);
    SE3.translation() = se3.bottomRows(3);

    return SE3;
}

Vector6d Log(Isometry3d const& SE3) {
    Vector3d const so3{Log(SE3.linear())};

    Vector6d pose;
    pose << so3, SE3.translation();

    return pose;
}

Matrix3d Exp(Vector3d const& so3) {
    Matrix3d const SO3{Eigen::AngleAxisd(so3.norm(), so3.normalized()).toRotationMatrix()};

    return SO3;
}

Vector3d Log(Matrix3d const& SO3) {
    Eigen::AngleAxisd const rotation(SO3);
    Vector3d const so3{rotation.angle() * rotation.axis()};

    return so3;
}

// TODO(Jack): Put this into a utility file if we find the right place?
std::vector<Isometry3d> ToSE3(std::vector<Array6d> const& se3) {
    std::vector<Isometry3d> SE3;
    SE3.reserve(std::size(se3));

    std::transform(std::cbegin(se3), std::cend(se3), std::back_inserter(SE3),
                   [](Array6d const& se3_i) { return Exp(Vector6d{se3_i}); });

    return SE3;
}

}  // namespace reprojection::geometry