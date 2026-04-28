#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// Adopted from
// https://stackoverflow.com/questions/62696455/is-there-a-way-to-find-the-median-value-of-coefficients-of-an-eigen-matrix
double Median(ArrayXd data);

}  // namespace reprojection::eigen_utilities