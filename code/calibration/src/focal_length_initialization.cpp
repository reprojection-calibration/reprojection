#include "calibration/focal_length_initialization.hpp"

namespace reprojection::calibration {

// Modified Least Squares method (MLS) from "A Few Methods for Fitting Circles to Data" Dale Umbach, Kerry N. Jones
std::tuple<Eigen::Vector2d, double> FitCircle(Eigen::MatrixX2d const& data) {
    Eigen::VectorXd const& x(data.col(0));
    Eigen::VectorXd const& y(data.col(1));
    Eigen::Index const n{data.rows()};

    double const xx{x.dot(x)};
    const double yy{y.dot(y)};
    double const sum_x{x.sum()};
    double const sum_y{y.sum()};

    double const A{n * xx - std::pow(sum_x, 2)};   // II.10
    double const B{n * x.dot(y) - sum_x * sum_y};  // II.11
    double const C{n * yy - std::pow(sum_y, 2)};   // II.12
    // TODO(Jack): Find a more expressive way to calculate the third power.
    double const D{0.5 * (n * x.dot(y.array().square().matrix()) - sum_x * yy +
                          n * (x.array() * x.array() * x.array()).sum() - sum_x * xx)};  // II.13
    double const E{0.5 * (n * y.dot(x.array().square().matrix()) - sum_y * xx +
                          n * (y.array() * y.array() * y.array()).sum() - sum_y * yy)};  // II.14

    double const denominator{A * C - B * B};
    double const cx{(D * C - B * E) / denominator};  // II.8
    double const cy{(A * E - B * D) / denominator};  // II.9

    Eigen::VectorXd const r{
        ((x.array() - cx) * (x.array() - cx) + (y.array() - cy) * (y.array() - cy)).sqrt()};  // II.15 (part)

    return {{cx, cy}, r.mean()};
}

}  // namespace reprojection::calibration