#include "focal_length_initialization.hpp"

namespace reprojection::calibration {

std::optional<double> EstimateFocalLength(Eigen::MatrixX2d const& pixels1, Eigen::MatrixX2d const& pixels2) {
    auto const circle1{FitCircle(pixels1)};
    auto const circle2{FitCircle(pixels2)};
    if (not (circle1.has_value() and circle2.has_value())) {
        return std::nullopt;
    }

    auto const vanishing_points{CircleCircleIntersection(circle1.value(), circle2.value())};
    if (not vanishing_points.has_value()) {
        return std::nullopt;
    }

    auto const [w1, w2]{vanishing_points.value()};
    double const f{(w1 - w2).norm() / M_PI};  // Eqn. 8 from reference "Equidistant Fish-Eye Calibration ..."

    return f;
}

// TODO(Jack): Do float math better and more expressive in the last condition - here and everywhere!
std::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> CircleCircleIntersection(Circle const& c1,
                                                                                     Circle const& c2) {
    auto const [P0, r0]{c1};
    auto const [P1, r1]{c2};

    double const d{(P1 - P0).norm()};
    if (d > r0 + r1) {
        return std::nullopt;  // Do not overlap at all
    } else if (d < std::abs(r0 - r1)) {
        return std::nullopt;  // One is inside the other
    } else if (d < 1e-8 and std::abs(r0 - r1) < 1e-8) {
        return std::nullopt;  // Coincident circles - infinite number of solutions
    }

    double const a{(r0 * r0 - r1 * r1 + d * d) / (2 * d)};
    double const h{std::sqrt(r0 * r0 - a * a)};
    if (h < 1e-8) {
        return std::nullopt;  // Circles intersect at one point, maybe generally interesting but not for us calibrating
    }

    Eigen::Vector2d const P2{P0 + a * (P1 - P0) / d};

    // TODO(Jack): Can we eloquently eliminate the copy and paste here? It is all done just to swap the signs for the
    // points, but maybe this is ok here.
    Eigen::Vector2d const P3_i{P2(0) + h * (P1(1) - P0(1)) / d, P2(1) - h * (P1(0) - P0(0)) / d};
    Eigen::Vector2d const P3_j{P2(0) - h * (P1(1) - P0(1)) / d, P2(1) + h * (P1(0) - P0(0)) / d};

    return std::tuple<Eigen::Vector2d, Eigen::Vector2d>{P3_i, P3_j};
}

// Modified Least Squares method (MLS) from "A Few Methods for Fitting Circles to Data" Dale Umbach, Kerry N. Jones
std::optional<Circle> FitCircle(Eigen::MatrixX2d const& data) {
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
    // TODO(Jack): We still need to figure out how line like is too generate for this to work and how we can catch that
    // error. I.e. what is the epsilon for the error condition on the denominator here.
    if (denominator < 1e-8) {
        return std::nullopt;
    }

    double const cx{(D * C - B * E) / denominator};  // II.8
    double const cy{(A * E - B * D) / denominator};  // II.9

    Eigen::VectorXd const r{
        ((x.array() - cx) * (x.array() - cx) + (y.array() - cy) * (y.array() - cy)).sqrt()};  // II.15 (part)

    return Circle{{cx, cy}, r.mean()};
}

}  // namespace reprojection::calibration