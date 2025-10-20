#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

#include <fstream>   // SAVING POINTS REMOVE
#include <iostream>  // SAVING POINTS REMOVE

#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

// Function to save a vector of Eigen::Isometry3d transformations to a CSV file
void saveTransformsToCSV(const std::vector<Eigen::Isometry3d>& transforms, const std::string& filename) {
    // Open the file to write the matrix
    std::ofstream file(filename);

    // Check if the file is open
    if (file.is_open()) {
        // Iterate over each transform in the vector
        for (const auto& transform : transforms) {
            // Write the 4x4 matrix of each transform on a new row
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    file << transform.matrix()(i, j);
                    if ( not (i * j == 9)) file << ",";  // Add a comma if not the last column
                }
            }
            file << "\n";  // Newline after tf
        }
        std::cout << "Transformation matrices saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file " << filename << "!" << std::endl;
    }
}

struct CameraTrajectory {
    Eigen::Vector3d world_origin;
    double sphere_radius;
    Eigen::Vector3d sphere_origin;

    // uint64_t t0_ns;
    // uint64_t tend_ns;
};

Eigen::Vector3d Cartesian(double const theta, double const phi) {
    double const x{std::sin(theta) * std::cos(phi)};
    double const y{std::sin(theta) * std::sin(phi)};
    double const z{std::cos(theta)};

    return {x, y, z};
}

Eigen::MatrixX3d GenerateSphere(double const radius, Eigen::Vector3d const origin) {
    int const points{100};
    int const loops{4};

    Eigen::MatrixX3d sphere(points, 3);
    for (int i{0}; i < points; ++i) {
        double const theta{2 * M_PI * loops * i / points};
        double const phi{2 * M_PI * i / points};

        sphere.row(i) = origin + (radius * Cartesian(theta, phi));
    }

    return sphere;
}

}  // namespace reprojection::testing_mocks

using namespace reprojection::testing_mocks;

TEST(TestingMocks, TestTrajectoryGenerator) {
    Eigen::MatrixX3d const sphere_points{GenerateSphere(0.5, {1, 1, 1})};

    std::vector<Eigen::Isometry3d> tfs;

    for (int i{0}; i < sphere_points.rows(); ++i) {
        Eigen::Isometry3d tf_i;
        tf_i.linear() = Eigen::Matrix3d::Identity();
        tf_i.translation() = sphere_points.row(i);

        tfs.push_back(tf_i);
    }

    saveTransformsToCSV(tfs, "sphere_cameras.txt");

    EXPECT_FALSE(true);
}