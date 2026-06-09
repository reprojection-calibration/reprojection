#include "optimization/extrinsic_optimization.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "steps/extrinsic_optimization.hpp"

namespace reprojection::steps {

std::string ExtrinsicOptimization::HashInputs() const {
    return hashing::HashArguments(camera_info, targets, intrinsics, imu_data, spline.ControlPoints(),
                                  spline.GetTimeHandler().t0_ns_, spline.GetTimeHandler().delta_t_ns_, extrinsic);
}

std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization::Compute() const {
    return optimization::ExtrinsicOptimization(imu_data, spline, extrinsic, camera_info, targets, intrinsics);
}

std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization::Load(SqlitePtr const db) const {
    // WARN(Jack): Here we are again hardcoding frame_b to be the camera frame!
    auto const control_points{database::ReadControlPoints(db, extrinsic.tf.frame_b, step_type)};
    auto const time_handler{database::ReadTimeHandler(db, extrinsic.tf.frame_b, step_type)};

    if (not time_handler) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicOptimization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }
    auto const optimized_spline{spline::Se3Spline{control_points, *time_handler}};

    auto const tf_imu_co{database::ReadExtrinsics(db, EntityId(), step_type)};
    auto const gravity_w{database::ReadGravity(db, EntityId(), step_type)};

    if (not tf_imu_co or not gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicOptimization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    ImuCamExtrinsic const optimized_extrinsic{*tf_imu_co, *gravity_w};

    return {optimized_spline, optimized_extrinsic};
}

void ExtrinsicOptimization::Save(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data, SqlitePtr const db) const {
    auto const [optimized_spline, optimized_extrinsic]{data};

    // WARN(Jack): Hardcoding frames to frame a/b! See note below!!!
    // ERROR(Jack): This is also unique here that we are inserting extra steps here in the save method! I do not think
    // these steps and their outputs will get removed if the main step entity cache busts. This is a danger!
    database::InsertStep(db, optimized_extrinsic.tf.frame_b, step_type, HashInputs());
    database::InsertControlPoints(db, optimized_extrinsic.tf.frame_b, step_type, optimized_spline.ControlPoints());
    database::InsertTimeHandler(db, optimized_extrinsic.tf.frame_b, step_type, optimized_spline.GetTimeHandler());

    auto const [spline_poses,
                errors]{optimization::ReprojectionErrorSpline(camera_info, targets, intrinsics, optimized_spline)};
    database::InsertPoses(db, optimized_extrinsic.tf.frame_b, step_type, spline_poses);
    database::InsertReprojectionErrors(db, optimized_extrinsic.tf.frame_b, step_type, errors);

    database::InsertExtrinsic(db, EntityId(), step_type, optimized_extrinsic.tf);
    database::InsertGravity(db, EntityId(), step_type, optimized_extrinsic.gravity);

    // TODO(Jack): Hardcoding the imu error to be saved under the entity_id of "frame_a" is a hacky hardcode! What we
    // want here is to make sure that these are saved under the entity_id of the imu which just so happens to be frame_a
    // but that might change! We also do this above with the camera stuff!
    ImuErrors const imu_error{optimization::EvaluateImuError(imu_data, optimized_extrinsic, optimized_spline)};
    database::InsertStep(db, optimized_extrinsic.tf.frame_b, step_type, HashInputs());
    database::InsertStep(db, optimized_extrinsic.tf.frame_a, step_type, HashInputs());
    database::InsertImuErrors(db, optimized_extrinsic.tf.frame_a, step_type, imu_error);
}

}  // namespace reprojection::steps
