#include "optimization/extrinsic_optimization.hpp"

#include "database/database_read.hpp"
#include "database/database_remove.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "steps/extrinsic_optimization.hpp"

namespace reprojection::steps {

std::string ExtrinsicOptimization::HashInputs() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_, imu_data_, spline_.ControlPoints(),
                                  spline_.GetTimeHandler().t0_ns_, spline_.GetTimeHandler().delta_t_ns_, extrinsic_);
}

std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization::Compute() const {
    return optimization::ExtrinsicOptimization(imu_data_, spline_, extrinsic_, camera_info_, targets_, intrinsics_,
                                               num_threads_);
}

std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization::Load(SqlitePtr const db) const {
    // WARN(Jack): Here we are again hardcoding frame_b to be the camera frame!
    auto const control_points{database::ReadControlPoints(db, extrinsic_.tf.frame_b, StepType())};
    auto const time_handler{database::ReadTimeHandler(db, extrinsic_.tf.frame_b, StepType())};

    if (not time_handler) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicOptimization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }
    auto const optimized_spline{spline::Se3Spline{control_points, *time_handler}};

    auto const tf_imu_co{database::ReadExtrinsics(db, EntityId(), StepType())};
    auto const gravity_w{database::ReadGravity(db, EntityId(), StepType())};

    if (not tf_imu_co or not gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicOptimization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    ImuCamExtrinsic const optimized_extrinsic{*tf_imu_co, *gravity_w};

    return {optimized_spline, optimized_extrinsic};
}

void ExtrinsicOptimization::Save(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data, SqlitePtr const db) const {
    auto const [optimized_spline, optimized_extrinsic]{data};

    // TODO(Jack): Hardcoding the imu error to be saved under the entity_id of "frame_a" is a hacky hardcode! What we
    // want here is to make sure that these are saved under the entity_id of the imu which just so happens to be frame_a
    // but that might change! We also do this above with the camera stuff!
    std::string const imu_name{optimized_extrinsic.tf.frame_a};
    std::string const camera_name{optimized_extrinsic.tf.frame_b};

    // TODO(Jack): This is some hacking here! The problem is that the spline and reprojection errors are being saved
    // here under the camera_name and not the EntityId() so we manually remove the step and insert the step here.
    // Normally this is handled automatically by the step runner. This is a sign we are doing something wrong here -
    // either the step runner abstraction is not complete or this step itself is doing too much. Regardless this should
    // "work" for now and now leave outdated rows in the db, but we need to keep out eyes peeled and make sure.
    database::RemoveFromDb(db, camera_name, StepType());
    database::InsertStep(db, camera_name, StepType(), HashInputs());
    database::InsertControlPoints(db, camera_name, StepType(), optimized_spline.ControlPoints());
    database::InsertTimeHandler(db, camera_name, StepType(), optimized_spline.GetTimeHandler());

    auto const [poses, reprojection_error]{
        optimization::ReprojectionErrorSpline(camera_info_, targets_, intrinsics_, optimized_spline)};
    database::InsertPoses(db, camera_name, StepType(), poses);
    database::InsertReprojectionErrors(db, camera_name, StepType(), reprojection_error);

    database::InsertExtrinsic(db, EntityId(), StepType(), optimized_extrinsic.tf);
    database::InsertGravity(db, EntityId(), StepType(), optimized_extrinsic.gravity);

    ImuErrors const imu_error{optimization::EvaluateImuError(imu_data_, optimized_extrinsic, optimized_spline)};
    database::InsertImuErrors(db, EntityId(), StepType(), imu_name, imu_error);
}

}  // namespace reprojection::steps
