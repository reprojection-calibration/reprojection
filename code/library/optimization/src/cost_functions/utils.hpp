#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization::cost_functions {

// Ceres tracks parameter blocks by their pointer address. Therefore, that requires that we pass parameters to the cost
// functions as plain pointers. The rest of our code base usually wants Eigen arrays/matrices and therefore we need
// to then map these pointers to more useful data type using Eigen::Map. This function here is a special case of this
// logic for the spline related cost functions. It showed up in several places so we put this together into a templated
// function.
//
// The template parameter T is the actual type of the array (almost always double or ceres::Jet) and N represents the
// size of the control points. For our case almost always just 3 or 6 depending if we are working with part of the pose
// or the entire pose.
template <typename T, int N>
Eigen::Matrix<T, N, spline::constants::order> BuildP(T const* const cp_0_ptr, T const* const cp_1_ptr,
                                                     T const* const cp_2_ptr, T const* const cp_3_ptr) {
    std::array<T const* const, spline::constants::order> const ptrs{cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr};

    Eigen::Matrix<T, N, spline::constants::order> P;
    for (int i{0}; i < spline::constants::order; ++i) {
        P.col(i) = Eigen::Map<Eigen::Vector<T, N> const>(ptrs[i], N, 1);
    }

    return P;
}

}  // namespace reprojection::optimization::cost_functions
