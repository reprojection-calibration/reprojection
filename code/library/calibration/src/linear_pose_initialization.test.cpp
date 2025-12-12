#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"

namespace reprojection::calibration {}  // namespace reprojection::calibration

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestXxxx) {
    std::string const record_path{"/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    auto const cam0_data{database::GetExtractedTargetData(db, "/cam0/image_raw")};
    ASSERT_TRUE(cam0_data.has_value());

    std::set<database::PoseData> cam0_pnp_poses;
    for (auto const& [timestamp_ns, extracted_target] : cam0_data.value()) {
        Array6d const cam0_ds_intrinsics{156.82590211, 156.79756958, 254.99978685,
                                         256.9744566,  -0.17931409,  0.59133716};
        auto const cam0_ds{projection_functions::DoubleSphereCamera(cam0_ds_intrinsics)};
        MatrixX3d const rays{cam0_ds.Unproject(extracted_target.bundle.pixels)};

        // ERROR(Jack): We are not accounting for the fact of valid field of view!
        static Array4d const pinhole_intrinsics{1, 1, 0, 0};
        auto const pinhole_camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
        MatrixX2d const pixels{pinhole_camera.Project(rays)};

        Bundle const linearized_target{pixels, extracted_target.bundle.points};

        auto const result{pnp::Pnp(linearized_target)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(result));

        Isometry3d const pose_i{std::get<Isometry3d>(result)};
        Vector6d se3_i{geometry::Log(pose_i)};

        // TODO(Jack): There has to be a better way to do this? Maybe just hardcode se3_n_1
        if (std::size(cam0_pnp_poses) <= 1) {
            cam0_pnp_poses.insert(database::PoseData{timestamp_ns, se3_i});
            continue;
        }

        database::PoseData const pose_n_1{*std::prev(cam0_pnp_poses.end())};
        if (se3_i.topRows<3>().dot(pose_n_1.pose.topRows<3>()) < 0) {
            se3_i.topRows<3>() *= -1;
        }

        cam0_pnp_poses.insert(database::PoseData{timestamp_ns, se3_i});
    }

    ASSERT_TRUE(AddInitialCameraPoseData("/cam0/image_raw", cam0_pnp_poses, db));
}