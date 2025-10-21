#include "calibration/focal_length_initialization.hpp"

namespace reprojection::calibration {

// Modified Least Squares method (MLS) from "A Few Methods for Fitting Circles to Data" Dale Umbach, Kerry N. Jones
std::tuple<Eigen::Vector2d, double> FitCircle(Eigen::MatrixX2d const& data) {
    Eigen::VectorXd const& x(data.col(0));
    Eigen::VectorXd const& y(data.col(1));
    Eigen::Index const n{data.rows()};

    // TODO DONT RECALCULATE THINGS OR COPY PASTE
    double const A{n * x.dot(x) - std::pow(x.sum(), 2)};  // II.10
    double const B{n * x.dot(y) - x.sum() * y.sum()};     // II.11
    double const C{n * y.dot(y) - std::pow(y.sum(), 2)};  // II.12
    // TODO THERE HAS TO BE A MORE ELOQUENT WAY TO RAISE TO THE THIRD POWER
    double const D{0.5 * (n * x.dot(y.array().square().matrix()) - x.sum() * y.dot(y) +
                          n * (x.array() * x.array() * x.array()).sum() - x.sum() * x.dot(x))};  // II.13
    double const E{0.5 * (n * y.dot(x.array().square().matrix()) - y.sum() * x.dot(x) +
                          n * (y.array() * y.array() * y.array()).sum() - y.sum() * y.dot(y))};  // II.14

    double const cx{(D * C - B * E) / (A * C - B * B)};  // II.8
    double const cy{(A * E - B * D) / (A * C - B * B)};  // II.9

    Eigen::VectorXd const r{
        ((x.array() - cx) * (x.array() - cx) + (y.array() - cy) * (y.array() - cy)).sqrt()};  // II.15 (part)

    return {{cx, cy}, r.mean()};
}

}  // namespace reprojection::calibration