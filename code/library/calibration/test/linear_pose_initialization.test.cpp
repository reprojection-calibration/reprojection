#include "calibration/linear_pose_initialization.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestLinearPoseInitializationDoubleSphere) {
    // Setup test data
    CameraInfo const sensor{"", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    CameraState const intrinsics{Array6d{600, 600, 360, 240, 0.1, 0.2}};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(sensor, intrinsics, 50, 1e9)};

    // Act
    OptimizationState const linear_solution{calibration::LinearPoseInitialization(sensor, targets, intrinsics)};

    // Assert
    EXPECT_EQ(std::size(linear_solution.frames), 50);
    for (auto const& [timestamp_ns, frame_i] : linear_solution.frames) {
        Array6d gt_aa_co_w{gt_frames.at(timestamp_ns).pose};

        EXPECT_TRUE(frame_i.pose.isApprox(gt_aa_co_w, 1e-12)) << "Linear pose initialization result:\n"
                                                              << frame_i.pose.transpose() << "\nGround truth:\n"
                                                              << gt_aa_co_w.transpose();
    }
}
