#include <gtest/gtest.h>

#include "calibration/linear_pose_initialization.hpp"
#include "calibration_data_views/initialization_view.hpp"
#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(Xxxx, Yyyyy) {
    float const timespan_ns{20e9};
    auto db{std::make_shared<database::CalibrationDatabase>("/tmp/reprojection/code/test_data/a_testing_mock.db3", true,
                                                            false)};

    // Camera data
    CameraCalibrationData cam_data{testing_mocks::GenerateMvgData(1000, timespan_ns, CameraModel::Pinhole,
                                                                  testing_utilities::pinhole_intrinsics,
                                                                  testing_utilities::image_bounds, true)};

    for (auto& [_, frame_i] : cam_data.frames) {
        frame_i.initial_pose = std::nullopt;
    }

    for (auto const& [timestamp_ns, frame_i] : cam_data.frames) {
        FrameHeader const header{timestamp_ns, cam_data.sensor.sensor_name};
        database::AddImage(header, db);
        AddExtractedTargetData({header, frame_i.extracted_target}, db);
    }

    std::cout << "calibration::LinearPoseInitialization(InitializationDataView(cam_data));" << std::endl;
    calibration::LinearPoseInitialization(InitializationDataView(cam_data));

    std::cout << "optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));" << std::endl;
    optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));

    std::cout << cam_data.optimized_intrinsics.transpose() << std::endl;

    for (auto& [_, frame_i] : cam_data.frames) {
        frame_i.initial_pose = geometry::Log(geometry::Exp(frame_i.initial_pose.value()).inverse());
        frame_i.optimized_pose = geometry::Log(geometry::Exp(frame_i.optimized_pose.value()).inverse());
    }

    // Write everything to database
    AddCameraPoseData(cam_data, database::PoseType::Initial, db);
    AddCameraPoseData(cam_data, database::PoseType::Optimized, db);
    database::AddReprojectionError(cam_data, database::PoseType::Initial, db);
    database::AddReprojectionError(cam_data, database::PoseType::Optimized, db);

    EXPECT_EQ(1, 2);
}