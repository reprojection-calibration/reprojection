#include "multiple_view_geometry_data_generator.hpp"

#include "pose_utilities.hpp"

namespace reprojection::pnp {

MvgFrameGenerator::MvgFrameGenerator()
    : points_{Eigen::MatrixX3d::Random(50, 3)}, K_{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}} {}

MvgFrame MvgFrameGenerator::Generate() const {
    // Generate pose
    Eigen::Vector3d camera_position{Eigen::Vector3d::Random()};
    camera_position.normalize();
    double const sphere_radius{3.5};  // Picked so that the points fill up as much of the image as possible
    camera_position *= sphere_radius;

    Eigen::Vector3d const origin{0, 0, 0};
    Eigen::Vector3d const camera_direction{TrackPoint(origin, camera_position)};

    // TODO(Jack): Can we construct this directly or do we need to use << - this is done in more than one place.
    Se3 viewing_pose_w_co;
    viewing_pose_w_co << camera_direction, camera_position;

    // Project points
    Eigen::Isometry3d const tf_co_w{FromSe3(viewing_pose_w_co).inverse()};  // There is an inverse here!!!
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_, K_, tf_co_w)};

    return MvgFrame{ToSe3(tf_co_w), pixels, points_};
}

Eigen::Matrix3d MvgFrameGenerator::GetK() const { return K_; }

Eigen::Vector3d MvgFrameGenerator::TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position) {
    // WARN(Jack): In testing this function I found that when the x and y coordinates of the origin and camera_position
    // are the same, i.e. the points  are aligned along the z-plane, the algorithm returns nans. There is probably a
    // simple test we can use to check this condition to avoid the nans, but for now we will just take this risk and
    // hope that we never have the same x and y coordinates for both camera and origin. This current implementation also
    // does not explicitly handle the case where the two points are the same, I assume that takes some error handling to
    // prevent nans as well.
    Eigen::Vector3d const origin_direction{(origin - camera_position).normalized()};
    Eigen::Vector3d const camera_forward_direction{0, 0, 1};

    double const angle{std::acos(origin_direction.transpose() * camera_forward_direction)};

    Eigen::Vector3d const cross_product{camera_forward_direction.cross(origin_direction)};
    Eigen::Vector3d const axis{cross_product / cross_product.norm()};

    Eigen::Vector3d const tracking_direction{angle * axis};

    return tracking_direction;
}

Eigen::MatrixX2d MvgFrameGenerator::Project(Eigen::MatrixX3d const& points_w, Eigen::Matrix3d const& K,
                                            Eigen::Isometry3d const& tf_co_w) {
    // TODO(Jack): Do we need to transform isometries into matrices before we use them? Otherwise it might not match our
    // expectations about matrix dimensions after the fact.
    // TODO(Jack): Should we use the pinhole projection from the nonlinear refinement optimization here?
    Eigen::MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};
    Eigen::MatrixX2d const pixels{(K * points_homog_co.leftCols(3).transpose()).transpose().rowwise().hnormalized()};

    return pixels;
}

}  // namespace reprojection::pnp
