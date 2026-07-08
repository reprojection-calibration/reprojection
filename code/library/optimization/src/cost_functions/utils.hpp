#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization::cost_functions {

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
