#include "optimization/extrinsic_optimization.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "spline/spline_initialization.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(OptimizationExtrinsicOptimization, TestExtrinsicOptimization) {
    uint64_t const timespan_ns{10000000000};
    CameraInfo const camera_info{"cam", CameraModel::Pinhole, testing_utilities::image_bounds};
    auto const [targets, camera_frames]{testing_mocks::GenerateMvgData(
        camera_info, CameraState{testing_utilities::pinhole_intrinsics}, 200, timespan_ns)};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(1000, timespan_ns)};

    spline::Se3Spline const spline{spline::InitializeSe3SplineState(camera_frames, 100)};

    std::string const imu_name{"imu"};
    ImuCamExtrinsic const initial_extrinsic{{imu_name, camera_info.sensor_name, Vector6d::Zero()}, Vector3d::Zero()};

    auto const [optimized_spline, optimized_extrinsic]{optimization::ExtrinsicOptimization(
        imu_data, spline, initial_extrinsic, camera_info, targets, {testing_utilities::pinhole_intrinsics})};

    std::cout << geometry::Exp(optimized_extrinsic.tf.se3_a_b).matrix() << std::endl;
    std::cout << optimized_extrinsic.gravity.transpose() << std::endl;

    std::string const record_path{"/tmp/reprojection/code/test_data/a1.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, true, false)};
    database::InsertEntity(db, camera_info.sensor_name, Entity::Camera);
    database::InsertEntity(db, imu_name, Entity::Imu);

    database::InsertImuData(db, imu_name, imu_data);
}

// See comments in TEST(OptimizationBundleAdjustment, TestEvaluateReprojectionResiduals) for context.
TEST(OptimizationExtrinsicOptimization, TestReprojectionErrorSpline) {
    MatrixX2d const gt_pixels{{-1, -1},  //
                              {350, 230},
                              {-1, -1},
                              {-1, -1},
                              {365, 245}};
    MatrixX3d const gt_points{{0, 0, -600},  //
                              {0, 0, 600},
                              {0, 0, -600},
                              {0, 0, -600},
                              {0, 0, 600}};
    ArrayX2d const gt_residuals{{256, 256},  //
                                {-10, -10},
                                {256, 256},
                                {256, 256},
                                {5, 5}};

    uint64_t const timestamp_ns{0};

    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraMeasurements const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};
    auto const camera_state{CameraState{testing_utilities::pinhole_intrinsics}};

    // The control points for a spline with one segment that is simply the constant identity transform.
    spline::Matrix2NK<double> control_points;
    control_points << Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero();
    spline::Se3Spline const spline{control_points, {0, 1}};

    auto const [poses, errors]{optimization::ReprojectionErrorSpline(sensor, targets, camera_state, spline)};
    EXPECT_EQ(std::size(poses), 1);
    EXPECT_TRUE(poses.at(timestamp_ns).pose.isApproxToConstant(0));
    EXPECT_EQ(std::size(errors), 1);
    EXPECT_TRUE(errors.at(timestamp_ns).isApprox(gt_residuals))
        << "Result:\n"
        << errors.at(timestamp_ns).transpose() << "\nexpected result:\n"
        << gt_residuals.transpose();
}

TEST(OptimizationExtrinsicOptimization, TestEvaluateImuError) {
    // TODO(Jack): Are we really sure that this test reflects the camera calibration case? In the camera calibration
    // case the trajectory is actually inversed (look at the mvg data generator). Lets try this on real data :)
    auto [imu_data, trajectory]{testing_mocks::GenerateImuData(100, 1'000'000'000)};

    ImuCamExtrinsic const extrinsic{Extrinsic{"imu", "cam", Array6d::Zero()}, Array3d::Zero()};
    auto const errors{optimization::EvaluateImuError(imu_data, extrinsic, trajectory)};

    EXPECT_EQ(std::size(errors), 100);
    for (auto const& error : errors) {
        EXPECT_TRUE(error.second.delta_angular_velocity.isApproxToConstant(0));
        EXPECT_TRUE(error.second.delta_linear_acceleration.isApproxToConstant(0));
    }
}