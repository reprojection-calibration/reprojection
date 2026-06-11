#include "new_sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): Can this be replaced with the ceres rotation functions I already have?
Matrix3d RotY(double const theta) {
    double const c{std::cos(theta)};
    double const s{std::sin(theta)};

    return Matrix3d{{c, 0, s}, {0, 1, 0}, {-s, 0, c}};
}

// TODO(Jack): Can this be replaced with the ceres rotation functions I already have?
Matrix3d RotZ(double const theta) {
    double const c{std::cos(theta)};
    double const s{std::sin(theta)};

    return Matrix3d{{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
}

Vector3d TrajectoryPosition(uint64_t const timestamp_ns, Vector3d const origin_w, double const radius) {
    const double timestamp_s{static_cast<double>(timestamp_ns / 1'000'000'000)};
    constexpr double speed_factor{0.1};

    // The main orbital rate - a speed_factor == 0.1 means that one full loop around the origin takes 10s.
    constexpr double orbit_rate{2.0 * M_PI * speed_factor};
    double const u{orbit_rate * timestamp_s};

    // The rate at which the orbital loops themselves rotate around the vertical axis - this prevents them from
    // overlapping. This is arbitrarily set to 1/10 the speed factor.
    constexpr double precession_rate{2.0 * M_PI * speed_factor / 10};
    double const q{precession_rate * timestamp_s};

    Matrix3d const R{RotZ(q) * RotY(0.5 * std::sin(q))};
    Vector3d const local{radius * Vector3d{0.0, std::cos(u), std::sin(u)}};

    return origin_w + (R * local);
}

}  // namespace reprojection::testing_mocks