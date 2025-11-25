#include <gtest/gtest.h>

#include <fstream>

#include "geometry/lie.hpp"
#include "sphere_trajectory.hpp"
#include "spline/se3_spline.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

bool writeSe3VectorToFile(const std::vector<Eigen::Matrix<double, 6, 1>>& poses, const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open file " << filename << "\n";
        return false;
    }

    for (const auto& p : poses) {
        out << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
    }

    out.close();
    return true;
}

spline::Se3Spline Se3Interpolate(std::vector<Vector6d> const& se3, std::vector<uint64_t> const& times) {
    std::vector<spline::C3Measurement> rotation_measurements;
    std::vector<spline::C3Measurement> translation_measurements;
    for (size_t i{0}; i < se3.size(); ++i) {
        Vector3d const so3{se3[i].topRows(3)};
        rotation_measurements.push_back(spline::C3Measurement{times[i], so3, spline::DerivativeOrder::Null});

        Vector3d const r3{se3[i].bottomRows(3)};
        translation_measurements.push_back(spline::C3Measurement{times[i], r3, spline::DerivativeOrder::Null});
    }

    int const num_segments{200};
    spline::CubicBSplineC3 const rotation_spline{
        spline::CubicBSplineC3Init::InitializeSpline(rotation_measurements, num_segments)};
    spline::CubicBSplineC3 const translation_spline{
        spline::CubicBSplineC3Init::InitializeSpline(translation_measurements, num_segments)};

    return spline::Se3Spline{translation_spline, rotation_spline};
}

TEST(XXX, FFF) {
    double const sphere_radius{1};
    Vector3d const sphere_origin{0, 0, 0};
    testing_mocks::CameraTrajectory const config{{0, 0, 2}, sphere_radius, sphere_origin};
    std::vector<Isometry3d> const poses{testing_mocks::SphereTrajectory(config)};

    uint64_t const t0_ns{1000000000};
    uint64_t const delta_t_ns{100000000};
    spline::Se3Spline se3_spline{t0_ns, delta_t_ns};
    for (auto const& pose : poses) {
        se3_spline.AddControlPoint(pose);
    }

    std::vector<Vector6d> se3;
    std::vector<uint64_t> times;
    int const num_evaluations{50};
    for (int i{0}; i < num_evaluations - 2; ++i) {
        uint64_t const t_i{
            static_cast<uint64_t>(delta_t_ns * std::size(poses) * (static_cast<double>(i) / num_evaluations))};
        auto const se3_i{se3_spline.EvaluateVec(t0_ns + t_i)};  // WARN UNPROTECTED OPTIONAL ACCESS!
        if (not se3_i.has_value()) {
            continue;
        }

        se3.push_back(se3_i.value());
        times.push_back(t_i);
    }

    writeSe3VectorToFile(se3, "sphere_trajectory.txt");

    // Do the interpolation
    auto const interpolated_spline{Se3Interpolate(se3, times)};

    std::vector<Vector6d> se3_interp_control_points;
    for (size_t i{0}; i < interpolated_spline.r3_spline_.control_points.size(); ++i) {
        Vector6d cp;
        cp.topRows(3) = interpolated_spline.so3_spline_.control_points[i];
        cp.bottomRows(3) = interpolated_spline.r3_spline_.control_points[i];

        se3_interp_control_points.push_back(cp);
    }

    writeSe3VectorToFile(se3_interp_control_points, "interpolated_sphere_trajectory.txt");
}
