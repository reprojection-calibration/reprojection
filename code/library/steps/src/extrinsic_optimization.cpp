#include "steps/extrinsic_optimization.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_imu_calibration.hpp"

namespace reprojection::steps {

std::string ExtrinsicOptimization::CacheKey() const {
    return caching::CacheKey(imu_data, spline.ControlPoints(), spline.GetTimeHandler().t0_ns_,
                             spline.GetTimeHandler().delta_t_ns_, tf_imu_co, gravity_w, camera_info, targets,
                             intrinsics);
}

// TODO(Jack): Use a real defined type?
std::tuple<spline::Se3Spline, Array6d, Array3d> ExtrinsicOptimization::Compute() const {
    auto const [optimized_spline, optimized_tf_imu_co, optimized_gravity_w, _]{
        optimization::ExtrinsicOptimization(imu_data, spline, tf_imu_co, gravity_w, camera_info, targets, intrinsics)};

    return {optimized_spline, optimized_tf_imu_co, optimized_gravity_w};
}

std::tuple<spline::Se3Spline, Array6d, Array3d> ExtrinsicOptimization::Load(SqlitePtr const db) const {
    auto const control_points{database::ReadControlPoints(db, SensorName(), step_type)};
    auto const time_handler{database::ReadTimeHandler(db, SensorName(), step_type)};
    // TODO(Jack): Naming! The _ prefix will get confusing for the future reader!
    auto const _tf_imu_co{database::ReadExtrinsics(db, SensorName(), step_type)};
    auto const _gravity_w{database::ReadGravity(db, SensorName(), step_type)};

    if (not time_handler or not _tf_imu_co or not _gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicOptimization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    return {spline::Se3Spline{control_points, *time_handler}, *_tf_imu_co, *_gravity_w};
}

void ExtrinsicOptimization::Save(std::tuple<spline::Se3Spline, Array6d, Array3d> const& state,
                                 SqlitePtr const db) const {
    // TODO(Jack): Naming! The _ prefix will get confusing for the future reader!
    auto const [_spline, _tf_imu_co, _gravity_w]{state};

    database::InsertControlPoints(db, SensorName(), step_type, _spline.ControlPoints());
    database::InsertTimeHandler(db, SensorName(), step_type, _spline.GetTimeHandler());
    database::InsertExtrinsic(db, SensorName(), step_type, _tf_imu_co);
    database::InsertGravity(db, SensorName(), step_type, _gravity_w);

    // ERROR(Jack): Hardcoding the IMU name here due to foreign key constraint!!!
    // Save Imu errors
    ImuErrors const imu_error{optimization::EvaluateImuError(imu_data, _spline, _tf_imu_co, _gravity_w)};
    database::InsertImuErrors(db, "/imu0", step_type, imu_error);

    // Save reprojection error
    auto const [spline_poses,
                reprojection_error]{optimization::ReprojectionErrorSpline(_spline, camera_info, targets, intrinsics)};
    database::InsertPoses(db, SensorName(), step_type, spline_poses);
    database::InsertReprojectionErrors(db, SensorName(), step_type, reprojection_error);
}

}  // namespace reprojection::steps
