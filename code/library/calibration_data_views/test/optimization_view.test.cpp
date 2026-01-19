#include "calibration_data_views/optimization_view.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CalibrationDataViews, TestOptimizationViewEmpty) {
    CameraCalibrationData data;
    OptimizationDataView data_view_1{data};

    bool empty{data_view_1.begin() == data_view_1.end()};
    ASSERT_TRUE(empty);

    // Add several frames that do not have an initial pose, which means they are skipped by the optimization view and
    // the view will still be considered empty.
    data.frames[0] = {};
    data.frames[1] = {};
    data.frames[2] = {};

    OptimizationDataView data_view_2{data};
    empty = {data_view_2.begin() == data_view_2.end()};
    ASSERT_TRUE(empty);
}

TEST(CalibrationDataViews, TestOptimizationViewFull) {
    CameraCalibrationData data;
    data.frames[0] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[1] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[2] = {{}, {0, 1, 2, 3, 4, 5}};

    // TODO(Jack): Using i is here how we do is a super hacky way to achieve our goal of measuring the container. If we
    // just had properly designed iterators than we could use standard tools for all of this.
    int i{0};

    OptimizationDataView data_view{data};
    for (OptimizationFrameView const& frame : data_view) {
        EXPECT_EQ(frame.timestamp_ns(), i);
        i++;
    }
    EXPECT_EQ(i, 3);
}

// The constructor initializes some values for the nonlinear optimization. If that stays here in the view we are not
// sure, but we need to test it!
TEST(CalibrationDataViews, TestOptimizationViewConstructorInitialization) {
    CameraCalibrationData data;
    data.initial_intrinsics = Array4d{0, 1, 2, 4};
    data.frames[0] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[1] = {{}, {0, 1, 2, 3, 4, 5}};
    data.frames[2] = {{}, {0, 1, 2, 3, 4, 5}};

    int i{0};

    OptimizationDataView data_view{data};
    EXPECT_TRUE(data.optimized_intrinsics.isApprox(data.initial_intrinsics));
    for (OptimizationFrameView frame : data_view) {
        ASSERT_TRUE(frame.optimized_pose().has_value());
        EXPECT_TRUE(frame.initial_pose().isApprox(frame.optimized_pose().value()));
        i++;
    }
    EXPECT_EQ(i, 3);
}