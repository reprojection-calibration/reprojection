#include <gtest/gtest.h>

#include "database/image_interface.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "testing_utilities/constants.hpp"

// TODO(Jack): Is there a file with this exact same name in the spline package?

namespace reprojection::calibration {}

using namespace reprojection;

TEST(CalibrationSplineInitialization, TestXxx) {
    testing_mocks::MvgGenerator const generator{CameraModel::Pinhole, testing_utilities::pinhole_intrinsics,
                                                testing_utilities::image_bounds};

    // NOTE(Jack): This should be less than the number of camera poses in the mvg sphere trajectory to prevent over
    // sampling which introduces artifacts.
    int const num_camera_poses{500};
    CameraCalibrationData const data{generator.GenerateBatch(num_camera_poses)};

    uint64_t const delta_t_ns{1000000};
    spline::Se3Spline se3_spline{0, delta_t_ns};

    // NOTE(Jack): We ignore the frame timestamp that comes from generate batch because we instead fix delta_t_ns above.
    // TODO(Jack): Add a spline construction interface which lets you add timestamped data and then initializes a
    //  uniform spline from the data.
    for (auto const& [_, frame_i] : data.frames) {
        // ERROR UNPROTECTED OPTIONAL ACCESS! Only acceptable because we are working on mvg testing mock data, but
        // should be fixed anyway.
        se3_spline.AddControlPoint(geometry::Exp(frame_i.initial_pose.value()));
    }

    // WARN UNDERSTAND/DOCUMENT THE "minus 3*interpolation_density" in the loop condition.
    int const interpolation_density{10};
    int const num_interpolation_points{interpolation_density * num_camera_poses};
    database::SplinePoses spline_poses;
    for (int i{0}; i < num_interpolation_points - 3 * interpolation_density; ++i) {
        // WARN(Jack): We ignore the starting time because we set t0_ns above to zero, if that changes later than set it
        // here too.
        uint64_t const time_i_ns{
            static_cast<uint64_t>(num_camera_poses * delta_t_ns * static_cast<double>(i) / num_interpolation_points)};
        Vector6d const pose_i{se3_spline.Evaluate(time_i_ns).value()};  // ERROR UNPROTECTED OPTIONAL ACCESS

        spline_poses[time_i_ns] = pose_i;
    }

    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};
    database::AddSplinePoseData(spline_poses, database::PoseType::Initial, db);

    // HACK to add the image frames and extracted targets for the non-existent extracted features/image foreign keys
    // relations
    for (auto const& [timestamp_ns, _] : data.frames) {
        FrameHeader const header_i{timestamp_ns, data.sensor.sensor_name};
        database::AddImage(header_i, db);
        database::AddExtractedTargetData({header_i, {}}, db);
    }
    database::AddCameraPoseData(data, database::PoseType::Initial, db);

    EXPECT_EQ(1, 2);
}