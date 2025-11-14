#include <ceres/ceres.h>

#include "spline/spline_evaluation_concept.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO REMOVE r3 idea
// NOTE(Jack): We purposely pick r3 as the variable name because it is generic enough to represent the idea that
// sometimes it is a value/posiiton, a velocity, or an acceleration depending on the derivative order.
template <typename T_Model, spline::DerivativeOrder D>
    requires spline::CanEvaluateCubicBSplineC3<T_Model>
class SplineCostFunction_T {
   public:
    SplineCostFunction_T(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns)
        : r3_{r3}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {
        // TODO(Jack): Where should we assert that u_i is between 0 and 1?
    }

    // NOTE(Jack): The reason that we need to pass each control point individually comes from ceres. Ceres does not
    // allow parameter blocks to partially overlap. If you do partially overlap you will get an error like,
    //
    //      1 problem_impl.cc:78] Check failed: !RegionsAlias( existing_block, existing_block_size, new_block,
    //      new_block_size)...
    //
    // My original intention was that we store the spline state (spline::CubicBSplineC3) therefore we will optimize on
    // the continuous memory representation as well and each residual will constrain the spline state like a sliding
    // window. Ceres does not allow partially overlapping parameter blocks like would be required by this sliding window
    // representation, therefore we have to break the parameter representation to a more granular form, and pass each
    // point individually. This allows ceres to recognize which parameters are actually the same and then group them
    // together for solving the problem.
    template <typename T>
    bool operator()(T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        // NOTE(Jack): We need to pass in the control points individually to satisfy ceres (see note above), but our
        // R3Spline::Evaluate() method takes a eigen matrix. Therefore, we have to organize them using maps
        // into points and then  place them in the control points matrix.
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(control_point_0_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(control_point_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p2(control_point_2_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p3(control_point_3_ptr);

        // TODO(Jack): Makes a copy! Can this be avoided? Technically, the underlying vector which the
        // control_point_*_ptr are passed in from should be continuous, and therefore we could just make a map taking
        // starting at control_point_0_ptr and map the next 12 elements into a Matrix3k. However this is entering
        // sketchy territory where we might start to violate the principle of least surprise. Unless there is some
        // benchmark showing this would help solve a problem, lets no do it :)
        spline::Matrix3K<T> control_points;
        control_points << p0, p1, p2, p3;

        Eigen::Vector<T, 3> const r3{T_Model::template Evaluate<T, D>(control_points, u_i_, delta_t_ns_)};

        residual[0] = T(r3_[0]) - r3[0];
        residual[1] = T(r3_[1]) - r3[1];
        residual[2] = T(r3_[2]) - r3[2];

        return true;
    }

    // NOTE(Jack): At this point we are really hardcoding that we are dealing with cubic splines here! The code can now
    // not be changed to any other spline degree without major rework.
    static ceres::CostFunction* Create(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineCostFunction_T, 3, 3, 3, 3, 3>(
            new SplineCostFunction_T(r3, u_i, delta_t_ns));
    }

    Vector3d r3_;
    double u_i_;
    std::uint64_t delta_t_ns_;
};

template <typename T_Model>
    requires spline::CanEvaluateCubicBSplineC3<T_Model>
ceres::CostFunction* CreateSplineCostFunction_T(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                double const u_i, std::uint64_t const delta_t_ns) {
    if (derivative == spline::DerivativeOrder::Null) {
        return SplineCostFunction_T<T_Model, spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::First) {
        return SplineCostFunction_T<T_Model, spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::Second) {
        return SplineCostFunction_T<T_Model, spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns);
    } else {
        throw std::runtime_error(
            "Requested unknown derivative order from CreateR3SplineCostFunction()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::optimization
