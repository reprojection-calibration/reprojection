#include "geometry/lie.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3) {
    Isometry3d SE3;

    Vector3d const so3{se3.topRows(3)};
    SE3.linear() = Exp<double>(so3);
    SE3.translation() = se3.bottomRows(3);

    return SE3;
}

Vector6d Log(Isometry3d const& SE3) {
    Vector3d const so3{Log<double>(SE3.linear())};

    Vector6d pose;
    pose << so3, SE3.translation();

    return pose;
}

}  // namespace reprojection::geometry