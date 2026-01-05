
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"

// ERROR NOT ACTUALLY ASSERTION OR TESTING STRATEGY
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>

using namespace reprojection;

Bundle Fov(Bundle const& bundle,MatrixX3d const& rays) {
    std::vector<Vector2d> pixels;
    std::vector<Vector3d> points;
    for (int i{0}; i < bundle.pixels.rows(); ++i) {
        Vector3d const point{rays.row(i)};

        double const xy_distance{std::sqrt(point(0) * point(0) + point(1) * point(1))};
        double const fov{std::acos(xy_distance) / point(2)};

        if (fov > M_PI / 3) {
            continue;
        }

        pixels.push_back(bundle.pixels.row(i));
        points.push_back(bundle.points.row(i));
    }

    Matrix2d pixels_eigen{std::size(pixels), 2};
    for (size_t i{0}; i < std::size(pixels); ++i) {
        pixels_eigen.row(i) = pixels[i];
    }
    Matrix3d points_eigen{std::size(points), 3};
    for (size_t i{0}; i < std::size(pixels); ++i) {
        points_eigen.row(i) = points[i];
    }

    return {pixels_eigen, points_eigen};
}

int main() {
    // ERROR(Jack): Hardcoded to work in clion
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    auto const cam0_data{database::GetExtractedTargetData(db, "/cam0/image_raw")};
    assert(cam0_data.has_value());

    Array6d const cam0_ds_intrinsics{156.82590211, 156.79756958, 254.99978685, 256.9744566, -0.17931409, 0.59133716};

    std::set<database::PoseStamped> cam0_pnp_poses;
    for (auto const& [header, extracted_target] : cam0_data.value()) {
        auto const cam0_ds{projection_functions::DoubleSphereCamera(cam0_ds_intrinsics)};
        MatrixX3d const rays{cam0_ds.Unproject(extracted_target.bundle.pixels)};

        // ERROR(Jack): We are not accounting for the fact of valid field of view!
        static Array4d const pinhole_intrinsics{1, 1, 0, 0};
        auto const pinhole_camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
        MatrixX2d const pixels{pinhole_camera.Project(rays)};

        Bundle const linearized_target{pixels, extracted_target.bundle.points};
        std::cout << "started fov" << std::endl;
        Bundle const fov_cleaned_target{Fov(linearized_target, rays)};
        std::cout << "cleaned fov" << std::endl;

        auto const result{pnp::Pnp(fov_cleaned_target)};
        if (not std::holds_alternative<Isometry3d>(result)) {
            continue;
        }

        Isometry3d const pose_i{std::get<Isometry3d>(result)};
        Vector6d se3_i{geometry::Log(pose_i)};

        // TODO(Jack): There has to be a better way to do this? Maybe just hardcode se3_n_1 as the forward z direction?
        if (std::size(cam0_pnp_poses) <= 1) {
            cam0_pnp_poses.insert(database::PoseStamped{{header.timestamp_ns, "/cam0/image_raw"}, se3_i});
            continue;
        }

        database::PoseStamped const pose_n_1{*std::prev(cam0_pnp_poses.end())};
        if (se3_i.topRows<3>().dot(pose_n_1.pose.topRows<3>()) < 0) {
            se3_i.topRows<3>() *= -1;
        }

        // WARN(Jack): We have to be in front of the target. This hardcodes the fact that we have a flat target with
        // all points at z=0. If we ever move to a world where this is not the case, and instead we allow multiple
        // targets or points not constrained to the z-plane, then we should actually check that the point z-depth in the
        // camera frame is correct. For not we avoid this slightly more complicated but also more robust check.
        if (se3_i(5) < 0) {
            se3_i.bottomRows<3>() *= -1;
        }

        cam0_pnp_poses.insert(database::PoseStamped{{header.timestamp_ns, "/cam0/image_raw"}, se3_i});
    }

    assert(AddPoseData(cam0_pnp_poses, database::PoseTable::Camera, database::PoseType::Initial, db));

    return EXIT_SUCCESS;
}