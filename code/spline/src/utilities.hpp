#pragma once

#include <Eigen/Dense>
#include <optional>
#include <tuple>

namespace reprojection::spline {

// Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph in
// section 4.2 Matrix Representation. In addition to the normalized segment time u we also return the segment index i as
// this is useful information for error/bounds checking and follows the "law of useful return" principle :)
std::tuple<double, int> NormalizedSegmentTime(uint64_t const t0_ns, uint64_t const t_ns, uint64_t const delta_t_ns);

// TODO(Jack): How can we also make this calculate the derivatives?
Eigen::VectorXd TimePolynomial(int const k, double const u, int const derivative);

Eigen::MatrixXd PolynomialCoefficients(int const k);

Eigen::MatrixXd BlendingMatrix(int const k);

Eigen::MatrixXd CumulativeBlendingMatrix(int const k);

// Note the symbol variables n and k come directly from wikipedia and are not chosen to reflect any relation to any
// other variable symbol in the library.
int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

// TODO(Jack): Where to define this?
// TODO(Jack): Naming
class TimeHandler {
   public:
    TimeHandler(uint64_t const t0_ns, uint64_t const delta_t_ns, int const k)
        : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns}, k_{k} {}

    std::optional<std::tuple<double, int>> SplinePosition(uint64_t const t_ns, size_t const num_knots) const {
        auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

        // From reference [1] - "At time t in [t_i, t_i+1) the value of p(t) only depends on the control points p_i,
        // p_i+1, ..., p_i+k-1" - See the start of the second paragraph in section 4.2 Matrix Representation.
        if (num_knots < static_cast<size_t>(i + k_)) {
            return std::nullopt;
        }

        return std::tuple{u_i, i};
    }

    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
    int k_;
};

}  // namespace reprojection::spline