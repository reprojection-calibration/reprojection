#include "trajectory.hpp"

#include "geometry/lie.hpp"

namespace reprojection::testing_mocks {

std::pair<Frames, ImuMeasurements> Trajectory(double const duration_s, double const sample_rate_hz,
                                              Vector3d const& origin_w, Vector3d const& target_w, double const radius) {
    auto const time_ns{SampleTimes(duration_s, sample_rate_hz)};

    // NOTE(Jack): We need to track the previous rotation to ensure that we make the smallest possible rotation in the
    // trajectory. If we do not do this there are cases along the trajectory where the orientation jumps significantly
    // due to discontinuities in the rotation math.
    std::optional<Matrix3d> R_w_b_prev;

    std::vector<Vector3d> p_w;
    std::vector<Matrix3d> R_w_b;
    for (auto const time_ns_i : time_ns) {
        Vector3d const p_w_i{PositionWorldBody(time_ns_i, origin_w, radius)};

        // NOTE(Jack): The "look at" rotation only really turns the y and z axes. The x-axis, which is our forward
        // direction remains pointing forward the entire time and therefore there is no roll effect. Because extrinsic
        // calibration requires that we excite all axes we manually add a roll about the x-axis here.
        Matrix3d const R_w_b_lookat{LookAtRotationWorldBody(p_w_i, target_w, R_w_b_prev)};
        Matrix3d const R_w_b_i{R_w_b_lookat * RollAboutBodyX(time_ns_i)};

        p_w.push_back(p_w_i);
        R_w_b.push_back(R_w_b_i);
        R_w_b_prev = R_w_b_i;
    }

    // TODO(Jack): Manually indexing into an array means we are probably doing something wrong. Is there really no
    // better way to get the time increment here?
    double const dt{static_cast<double>(time_ns[1] - time_ns[0]) / 1'000'000'000};

    std::map<uint64_t, Vector3d> omega_b;
    for (int i{2}; i < std::size(time_ns) - 2; ++i) {
        Matrix3d const R_dot_w{(R_w_b[i - 2] - 8.0 * R_w_b[i - 1] + 8.0 * R_w_b[i + 1] - R_w_b[i + 2]) / (12.0 * dt)};

        Matrix3d const R_b_w{R_w_b[i].inverse()};
        Matrix3d omega_hat_b{R_b_w * R_dot_w};
        omega_hat_b = 0.5 * (omega_hat_b - omega_hat_b.transpose());

        omega_b.insert({time_ns[i], Vee(omega_hat_b)});
    }

    std::map<uint64_t, Vector3d> acc_w;
    for (int i{2}; i < std::size(time_ns) - 2; ++i) {
        Vector3d const acc_w_i{(-p_w[i - 2] + 16 * p_w[i - 1] - 30 * p_w[i] + 16 * p_w[i + 1] - p_w[i + 2]) /
                               (12.0 * std::pow(dt, 2))};
        acc_w.insert({time_ns[i], acc_w_i});
    }

    std::map<uint64_t, Vector3d> specific_force_b;
    for (int i{2}; i < std::size(time_ns) - 2; ++i) {
        Matrix3d const R_b_w{R_w_b[i].inverse()};

        Vector3d const gravity_w{0.0, 0.0, -9.81};
        uint64_t const time_ns_i{time_ns[i]};

        Vector3d const specific_force_b_i{R_b_w * (acc_w.at(time_ns_i) - gravity_w)};
        specific_force_b.insert({time_ns_i, specific_force_b_i});
    }

    Frames frames;
    ImuMeasurements imu_measurements;
    for (int i{2}; i < std::size(time_ns) - 2; ++i) {
        Vector6d se3;
        se3 << geometry::Log(R_w_b[i]), p_w[i];

        uint64_t const time_ns_i{time_ns[i]};
        frames.insert({time_ns_i, {se3}});

        ImuMeasurement const imu_measurement_i{time_ns_i, {{omega_b.at(time_ns_i)}, {specific_force_b.at(time_ns_i)}}};
        imu_measurements.insert(imu_measurement_i);
    }

    return {frames, imu_measurements};
}

Vector3d PositionWorldBody(uint64_t const timestamp_ns, Vector3d const& origin_w, double const radius) {
    const double timestamp_s{static_cast<double>(timestamp_ns) / 1e9};
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
Matrix3d LookAtRotationWorldBody(Vector3d const& position_w, Vector3d const& target_w,
                                 std::optional<Matrix3d> const& R_w_b_prev) {
    Vector3d const target_delta_w{target_w - position_w};

    if (target_delta_w.norm() < 1e-8) {
        return R_w_b_prev.value_or(Matrix3d::Identity());
    }

    Vector3d const x_b_w_new{target_delta_w.normalized()};

    if (!R_w_b_prev.has_value()) {
        //
        // Initialize orientation from world_up.
        //

        Vector3d world_up{0.0, 0.0, 1.0};

        if (std::abs(x_b_w_new.dot(world_up)) > 0.95) {
            world_up = Vector3d{0.0, 1.0, 0.0};
        }

        Vector3d const y_b_w{world_up.cross(x_b_w_new).normalized()};

        Vector3d const z_b_w{x_b_w_new.cross(y_b_w).normalized()};

        Matrix3d R_w_b;
        R_w_b << x_b_w_new, y_b_w, z_b_w;

        return R_w_b;
    } else {
        //
        // Propagate previous orientation using the smallest rotation
        // that moves the previous forward axis onto the new forward axis.
        //

        Vector3d const x_b_w_prev{R_w_b_prev->col(0)};

        Vector3d const axis{x_b_w_prev.cross(x_b_w_new)};

        double const sin_angle{axis.norm()};

        double const cos_angle{std::clamp(x_b_w_prev.dot(x_b_w_new), -1.0, 1.0)};

        // No meaningful change.
        if (sin_angle < 1e-8 && cos_angle > 0.0) {
            return *R_w_b_prev;
        }

        // 180-degree flip case.
        if (sin_angle < 1e-8 && cos_angle < 0.0) {
            Vector3d axis_180{x_b_w_prev.cross(Vector3d::UnitZ())};

            if (axis_180.norm() < 1e-8) {
                axis_180 = x_b_w_prev.cross(Vector3d::UnitY());
            }

            Eigen::AngleAxisd const aa{M_PI, axis_180.normalized()};

            return aa.toRotationMatrix() * (*R_w_b_prev);
        }

        double const angle{std::atan2(sin_angle, cos_angle)};

        Eigen::AngleAxisd const aa{angle, axis.normalized()};

        Matrix3d const R_delta{aa.toRotationMatrix()};

        return R_delta * (*R_w_b_prev);
    }
}

Eigen::Array<uint64_t, -1, 1> SampleTimes(double const duration_s, double const sample_rate_hz) {
    int const num_samples{static_cast<int>(duration_s * sample_rate_hz)};
    uint64_t const duration_ns{static_cast<uint64_t>(1'000'000'000 * duration_s)};

    // TODO(Jack): Define type for Eigen::Array<uint64_t, -1, 1>?
    auto const times_ns{Eigen::Array<uint64_t, -1, 1>::LinSpaced(num_samples, 0, duration_ns)};

    return times_ns;
}

Matrix3d RollAboutBodyX(uint64_t const timestamp_ns) {
    // TODO(Jack): These are actually critical constants which control the trajectory. Hardcoding them here is a little
    // hacky, but honestly I can't imagine a world where we actually need to parameterize these things, therefore having
    // them hardcoded here close to the usage is a solution for now.
    double constexpr roll_amplitude{0.001};
    double constexpr roll_frequency_hz{0.1};

    double const timestamp_s{static_cast<double>(timestamp_ns) / 1e9};
    double const roll{roll_amplitude * std::sin(2.0 * M_PI * roll_frequency_hz * timestamp_s)};

    return geometry::Exp<double>({roll, 0, 0});
}

}  // namespace reprojection::testing_mocks