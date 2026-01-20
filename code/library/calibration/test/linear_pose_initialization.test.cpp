#include "calibration/linear_pose_initialization.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestLinearPoseInitialization) {
    testing_mocks::MvgGenerator const generator{
        CameraModel::DoubleSphere, Array6d{600, 600, 360, 240, 0.1, 0.2}, {0, 720, 0, 480}};
    CameraCalibrationData const gt_data{generator.GenerateBatch(20)};
    CameraCalibrationData data{gt_data};

    // Reset the initial poses to null to make sure we do not confuse them for the output of LinearPoseInitialization.
    for (auto& [_, frame_i] : data.frames) {
        frame_i.initial_pose = std::nullopt;
    }

    calibration::LinearPoseInitialization(InitializationDataView(data));

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Array6d const se3_gt_pose_i{gt_data.frames.at(timestamp_ns).initial_pose.value()};

        ASSERT_TRUE(frame_i.initial_pose.has_value());
        EXPECT_TRUE(frame_i.initial_pose.value().isApprox(se3_gt_pose_i, 1e-6))
            << "Linear pose initialization result:\n"
            << frame_i.initial_pose.value().transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }
}
