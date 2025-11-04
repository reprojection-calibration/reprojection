#include "noise_generation.hpp"

#include <random>

#include "geometry/lie.hpp"

namespace reprojection::testing_mocks {

Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose) {
    pose.translation() += GaussianNoise(0, sigma_translation, 3, 1);

    // TODO(Jack): Confirm this is the right way to add noise to a rotation! Adding the gaussian noise element in the
    // tangert space directly and then converting back to a rotation matrix.
    Vector3d const rotation_noise{GaussianNoise(0, sigma_rotation, 3, 1)};
    Vector3d const rotation_se3{geometry::Log(pose.rotation())};
    Vector3d const perturbed_se3{rotation_noise.array() + rotation_se3.array()};

    Matrix3d const perturbed_R{geometry::Exp(perturbed_se3)};
    pose.linear() = perturbed_R;

    return pose;
}

// Generates completely independent zero mean gaussian noise. There is no dependence in either the rows or columns!
MatrixXd GaussianNoise(double const mean, double const sigma, int const rows, int const cols) {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;

    MatrixXd noise{MatrixXd::Constant(rows, cols, mean)};
    noise = noise.unaryExpr([sigma](double const& x) { return x + sigma * dist(gen); });

    return noise;
}

}  // namespace reprojection::testing_mocks
