#include <Eigen/Dense>

namespace reprojection::pnp {

bool IsPlane(Eigen::MatrixX3d const& points);

// TODO(Jack): Not tested
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> Pca(Eigen::MatrixX3d const& points);

}  // namespace reprojection::pnp