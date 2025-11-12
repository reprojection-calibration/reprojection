#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// COPY AND PASTED FROM EXACT SAME AS FOR R3 case
template <spline::DerivativeOrder D>
class So3SplineCostFunction_T {
   public:
    So3SplineCostFunction_T(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns)
        : r3_{r3}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    template <typename T>
    bool operator()(T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(control_point_0_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(control_point_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p2(control_point_2_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p3(control_point_3_ptr);

        spline::Matrix3K<T> control_points;
        control_points << p0, p1, p2, p3;

        Eigen::Vector<T, 3> const r3{spline::So3SplineEvaluation::Evaluate<T, D>(control_points, u_i_, delta_t_ns_)};

        residual[0] = T(r3_[0]) - r3[0];
        residual[1] = T(r3_[1]) - r3[1];
        residual[2] = T(r3_[2]) - r3[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<So3SplineCostFunction_T, 3, 3, 3, 3, 3>(
            new So3SplineCostFunction_T(r3, u_i, delta_t_ns));
    }

    Vector3d r3_;
    double u_i_;
    std::uint64_t delta_t_ns_;
};

// Exact same as r3 case!
ceres::CostFunction* CreateSo3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                 double const u_i, std::uint64_t const delta_t_ns) {
    if (derivative == spline::DerivativeOrder::Null) {
        return So3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::First) {
        return So3SplineCostFunction_T<spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::Second) {
        return So3SplineCostFunction_T<spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns);
    } else {
        throw std::runtime_error(
            "Requested unknown derivative order from CreateR3SplineCostFunction()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::optimization

using namespace reprojection;

double Squared(double const x) { return x * x; }  // COPY PASTED!!!!

// Look in the so3 spline units test/r3 unit test to understand the testing values and philsophy.
TEST(OptimizationSo3SplineCostFunction, TestCreateSo3SplineCostFunction) {
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    spline::Matrix3Kd const P{{-1, -0.5, 0.5, 1},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};

    // Set up the required input data for the Evaluate() method (normally handled internally by ceres).
    double const* const P1_ptr{P.col(0).data()};
    double const* const P2_ptr{P.col(1).data()};
    double const* const P3_ptr{P.col(2).data()};
    double const* const P4_ptr{P.col(3).data()};
    double const* const P_ptr_ptr[4]{P1_ptr, P2_ptr, P3_ptr, P4_ptr};
    Vector3d residual;

    // Position
    Array3d const position{1.0319672855968482, -0.40184576778254827, -0.89024132986881044};
    ceres::CostFunction* cost_function{
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::Null, position, u_i, delta_t_ns)};
    bool success{cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr)};
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Velocity
    Array3d const velocity{0.8563971186898035, -0.1204280865611993, 0.12722122556164611};
    cost_function =
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::First, velocity, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Acceleration
    Array3d const acceleration{0.0069974409407700944, 0.80095289350156396, 0.71108131312833733};
    cost_function =
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::Second, acceleration, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());
}

TEST(OptimizationSo3SplineCostFunction, TestSo3SplineCostFunctionCreate_T) {
    Vector3d const so3{0, 0, 0};
    double const u_i{0.2};
    std::uint64_t const delta_t_ns{5};

    ceres::CostFunction const* const cost_function{
        optimization::So3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(so3, u_i, delta_t_ns)};

    // Four r3 control point parameter blocks of size three and a r3 residual
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 4);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 3);
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
