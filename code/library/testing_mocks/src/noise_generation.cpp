#include "noise_generation.hpp"

#include <random>

namespace reprojection::testing_mocks {

// Generates completely independent zero mean gaussian noise. There is no dependence in either the rows or columns!
MatrixXd GaussianNoise(double const mean, double const sigma, int const rows, int const cols) {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;

    MatrixXd noise{MatrixXd::Constant(rows, cols, mean)};
    noise = noise.unaryExpr([sigma](double const& x) { return x + sigma * dist(gen); });

    return noise;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::testing_mocks
