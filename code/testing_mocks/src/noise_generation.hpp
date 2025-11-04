#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose);

// Generates completely independent zero mean gaussian noise. There is no dependence in either the rows or columns!
MatrixXd GaussianNoise(double const mean, double const sigma, int const rows, int const cols);

}  // namespace reprojection::testing_mocks
