#include "calibration_data_views/optimization_view.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CalibrationDataViews, TestOptimizationViewEmpty) {
    CameraCalibrationData data;
    OptimizationDataView data_view_1{data};
    EXPECT_EQ(data_view_1.valid_frame_count(), 0);

    // Add several frames that do not have an initial pose, which means they are skipped by the optimization view and
    // the view will still be considered empty.
    data.frames[0] = {};
    data.frames[1] = {};
    data.frames[2] = {};

    OptimizationDataView data_view_2{data};
    EXPECT_EQ(data_view_2.valid_frame_count(), 0);
}

TEST(CalibrationDataViews, TestOptimizationView) {
    CameraCalibrationData data;
    OptimizationDataView data_view{data};
    data.frames[0] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[11] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[22] = {};
    data.frames[33] = {};
    data.frames[44] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[55] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[66] = {};

    EXPECT_EQ(data_view.valid_frame_count(), 4);

    std::vector<uint64_t> timestamps_ns;
    for (OptimizationFrameView frame : data_view) {
        timestamps_ns.push_back(frame.timestamp_ns());
    }
    EXPECT_EQ(std::size(timestamps_ns), 4);

    std::vector<int> gt_timestamps_ns{0, 11, 44, 55};
    EXPECT_TRUE(std::equal(timestamps_ns.begin(), timestamps_ns.end(), gt_timestamps_ns.begin()));
}

// The constructor initializes some values for the nonlinear optimization. If that stays here in the view we are not
// sure, but we need to test it!
TEST(CalibrationDataViews, TestOptimizationViewConstructorInitialization) {
    CameraCalibrationData data;
    data.initial_intrinsics = Array4d{0, 1, 2, 4};
    data.frames[0] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[1] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[2] = {{}, {0, 1, 2, 3, 4, 5}};

    OptimizationDataView data_view{data};
    EXPECT_EQ(data_view.valid_frame_count(), 3);

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(data.initial_intrinsics));
    for (OptimizationFrameView frame : data_view) {
        ASSERT_TRUE(frame.optimized_pose().has_value());
        EXPECT_TRUE(frame.initial_pose().isApprox(frame.optimized_pose().value()));
    }
}
