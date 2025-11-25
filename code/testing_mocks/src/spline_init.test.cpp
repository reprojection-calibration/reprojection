#include <gtest/gtest.h>

#include <fstream>

#include "geometry/lie.hpp"
#include "sphere_trajectory.hpp"
#include "spline/se3_spline.hpp"
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
    int const num_evaluations{90};
    for (int i{0}; i < num_evaluations -2; ++i) {
        uint64_t const t_i{
            static_cast<uint64_t>(delta_t_ns * std::size(poses) * (static_cast<double>(i) / num_evaluations))};
        // WARN UNPROTECTED OPTIONAL ACCESS!
        Vector6d const se3_i{se3_spline.EvaluateVec(t0_ns + t_i).value()};
        se3.push_back(se3_i);
    }

    writeSe3VectorToFile(se3, "sphere_trajectory.txt");
}
