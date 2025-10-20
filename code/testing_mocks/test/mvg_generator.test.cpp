#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

#include <fstream>   // SAVING POINTS REMOVE
#include <iostream>  // SAVING POINTS REMOVE

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
                    if (not(i * j == 9)) file << ",";  // Add a comma if not the last column
                }
            }
            file << "\n";  // Newline after tf
        }
        std::cout << "Transformation matrices saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file " << filename << "!" << std::endl;
    }
}

}  // namespace reprojection::testing_mocks

using namespace reprojection::testing_mocks;

TEST(TestingMocks, XXX) {
    auto const generator{MvgGenerator()};

    auto const frame{generator.Generate(0.1)};

    std::cout << frame.pixels << std::endl;

    EXPECT_FALSE(true);
}

TEST(TestingMocks, TestProject) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX2d const pixels{MvgGenerator::Project(points_w, K, tf_co_w)};

    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));
}