#include "sphere_trajectory.hpp"

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
    CameraTrajectory const config{{3, 3, 0}, 4, {0, 0, 0}};

    std::vector<Eigen::Isometry3d> const tfs{SphereTrajectory(config)};

    saveTransformsToCSV(tfs, "sphere_cameras.txt");

    EXPECT_FALSE(true);
}