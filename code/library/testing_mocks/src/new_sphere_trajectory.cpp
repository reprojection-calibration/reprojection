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

// TODO(Jack): Does this handle edge cases like the target and position being in the same coordinate plane?
Matrix3d LookAtRotationWorldBody(Vector3d const position_w, Vector3d const target_w) {
    Vector3d const x_b_w{(target_w - position_w).normalized()};

    if (x_b_w.norm() < 1e-3) {
        return Matrix3d::Identity();
    }

    Vector3d world_up{0, 0, 1};
    if (std::abs(x_b_w.dot(world_up)) < 0.95) {
        world_up = Vector3d{0, 1, 0};
    }

    Vector3d const y_b_w{world_up.cross(x_b_w).normalized()};
    Vector3d const z_b_w{world_up.cross(y_b_w).normalized()};

    // TODO(Jack): Check that this fills by columns and not rows!!!
    Matrix3d R_b_w;
    R_b_w << x_b_w, y_b_w, z_b_w;

    return R_b_w;
}

Eigen::Array<uint64_t, -1, 1> SampleTimes(double const duration_s, double const sample_rate_hz) {
    int const num_samples{static_cast<int>(duration_s * sample_rate_hz)};
    uint64_t const duration_ns{static_cast<uint64_t>(1'000'000'000 * duration_s)};

    // TODO(Jack): Define type for Eigen::Array<uint64_t, -1, 1>?
    auto const times_ns{Eigen::Array<uint64_t, -1, 1>::LinSpaced(num_samples, 0, duration_ns)};

    return times_ns;
}

}  // namespace reprojection::testing_mocks