
#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

namespace reprojection {}

using namespace reprojection;

TEST(Xxxx, Yyyyy) {
    auto db{std::make_shared<database::CalibrationDatabase>("/tmp/reprojection/code/test_data/a_testing_mock.db3", true,
                                                            false)};

    float const timespan_ns{20e9};

    // Camera data
    CameraCalibrationData const camera_data{
        testing_mocks::GenerateMvgData(100, timespan_ns, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics,
                                       testing_utilities::image_bounds, false)};
    for (auto const& [timestamp_ns, frame_i] : camera_data.frames) {
        FrameHeader const header{timestamp_ns, camera_data.sensor.sensor_name};
        database::AddImage(header, db);
        AddExtractedTargetData({header, {}}, db);
    }
    database::AddCameraPoseData(camera_data, database::PoseType::Initial, db);

    // Imu data
    // TODO MAKE THE IMU FUNCTION ACCEPT THE IMU DATA TYPE - FIRST INVENT THAT TYPE!
    testing_mocks::ImuData const imu_data{testing_mocks::GenerateImuData(1000, timespan_ns)};
    for (auto const& [timestamp_ns, measurement_i] : imu_data) {
        ImuStamped const temp_i{{timestamp_ns, "/mvg_imu"}, measurement_i};
        (void)database::AddImuData(temp_i, db);
    }

    EXPECT_EQ(1, 2);
}
