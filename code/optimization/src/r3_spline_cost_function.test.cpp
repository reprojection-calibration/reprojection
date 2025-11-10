#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): Template to handle velocity and acceleration also
// TODO(Jack): We purposely pick r3 as the variable name because it is generic enough to represent the idea that
// sometimes it is a value, a velocity, or an acceleration depending on the cost function.
template <spline::DerivativeOrder D>
class R3SplineCostFunction {
   public:
    R3SplineCostFunction(Vector3d const& r3, double const u_i) : r3_{r3}, u_i_{u_i} {}

    template <typename T>
    bool operator()(T const* const control_points_ptr, T* const residual) const {
        Eigen::Map<Eigen::Matrix<T, 3, spline::constants::order> const> control_points(control_points_ptr);
        Eigen::Vector<T, 3> const r3{spline::R3SplineEvaluation::Evaluate<T, D>(control_points, T(u_i_))};

        residual[0] = T(r3_[0]) - r3[0];
        residual[1] = T(r3_[1]) - r3[1];
        residual[2] = T(r3_[2]) - r3[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& r3, double const u_i) {
        return new ceres::AutoDiffCostFunction<R3SplineCostFunction, 3, 3 * spline::constants::order>(
            new R3SplineCostFunction(r3, u_i));
    }

    Vector3d r3_;
    double u_i_;
};

}  // namespace reprojection::optimization

using namespace reprojection;

TEST(OptimizationR3SplineCostFunction, TestXXX) {
    Vector3d const r3{0, 0, 0};
    double const u_i{0.2};
    ceres::CostFunction const* const cost_function{
        optimization::R3SplineCostFunction<spline::DerivativeOrder::Null>::Create(r3, u_i)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 1);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 12);
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
