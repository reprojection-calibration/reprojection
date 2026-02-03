
#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/spline_state.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

namespace reprojection::calibration {

using CubicBSplineC3 = spline::CubicBSplineC3;
using C3Measurement = spline::C3Measurement;
using DerivativeOrder = spline::DerivativeOrder;
using CubicBSplineC3Init = spline::CubicBSplineC3Init;

// TODO(Jack): This should really only get a view of the optimized poses as that is all it needs!
std::tuple<CubicBSplineC3, CubicBSplineC3> InitSpline(CameraFrameSequence const& camera_frames,
                                                      int const num_interpolation_points) {
    std::vector<C3Measurement> rotation_measurements;
    rotation_measurements.reserve(std::size(camera_frames));
    std::vector<C3Measurement> translation_measurements;
    translation_measurements.reserve(std::size(camera_frames));

    for (auto const& [timestamp_ns, frame_i] : camera_frames) {
        // ERROR UNPROTECTED OPTIONAL ACCESS!
        rotation_measurements.push_back({timestamp_ns, frame_i.initial_pose.value().topRows(3), DerivativeOrder::Null});
        translation_measurements.push_back(
            {timestamp_ns, frame_i.initial_pose.value().bottomRows(3), DerivativeOrder::Null});
    }

    CubicBSplineC3 const rotation_spline{
        CubicBSplineC3Init::InitializeSpline(rotation_measurements, num_interpolation_points)};
    CubicBSplineC3 const translation_spline{
        CubicBSplineC3Init::InitializeSpline(translation_measurements, num_interpolation_points)};

    return {rotation_spline, translation_spline};
}

}  // namespace reprojection::calibration

using namespace reprojection;

TEST(Xxxx, Yyyyy) {
    float const timespan_ns{20e9};

    // Camera data
    CameraCalibrationData const camera_data{testing_mocks::GenerateMvgData(100, timespan_ns, CameraModel::Pinhole,
                                                                           testing_utilities::pinhole_intrinsics,
                                                                           testing_utilities::image_bounds, false)};

    auto const [rot_spline, trans_spline]{calibration::InitSpline(camera_data.frames, 2000)};

    CameraCalibrationData  interpolated_spline;
    interpolated_spline.sensor.sensor_name= "/interpolated_spline";
    for (uint64_t i{0}; i< std::size(rot_spline.control_points); ++i) {
        Vector6d pose_i;
        pose_i.topRows(3) = rot_spline.control_points[i];
        pose_i.bottomRows(3) = trans_spline.control_points[i];
        interpolated_spline.frames[i].initial_pose =  pose_i;
    }

    auto db{std::make_shared<database::CalibrationDatabase>("/tmp/reprojection/code/test_data/a_testing_mock.db3", true,
                                                        false)};

    for (auto const& [timestamp_ns, frame_i] : camera_data.frames) {
        FrameHeader const header{timestamp_ns, camera_data.sensor.sensor_name};
        database::AddImage(header, db);
        AddExtractedTargetData({header, {}}, db);
    }
    database::AddCameraPoseData(camera_data, database::PoseType::Initial, db);

    for (auto const& [timestamp_ns, frame_i] : interpolated_spline.frames) {
        FrameHeader const header{timestamp_ns, interpolated_spline.sensor.sensor_name};
        database::AddImage(header, db);
        AddExtractedTargetData({header, {}}, db);
    }
    database::AddCameraPoseData(interpolated_spline, database::PoseType::Initial, db);




    EXPECT_EQ(1, 2);
}
