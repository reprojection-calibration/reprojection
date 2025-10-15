#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

// TOOD(Jack): Is it ok to do this kind of defining in non-public header files?
using Se3 = Eigen::Vector<double, 6>;

Se3 ToSe3(Eigen::Isometry3d const& matrix);

Eigen::Isometry3d FromSe3(Se3 const& se3);

}  // namespace reprojection_calibration::pnp