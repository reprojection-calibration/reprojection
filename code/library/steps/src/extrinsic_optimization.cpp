#include "optimization/extrinsic_optimization.hpp"

#include <ranges>

#include "database/database_read.hpp"
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

ImuMeasurements CalibrateImuData(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data,
                                 ImuMeasurements const& imu_data) {
    auto const& [spline_w_co, extrinsic]{data};

    ImuMeasurements calibrated_imu_data;
    for (auto const timestamp_ns : imu_data | std::views::keys) {
        auto const se3_w_co{spline_w_co.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        if (not se3_w_co) {
            continue;
        }
        Matrix3d const R_w_co{geometry::Exp<double>(se3_w_co->topRows<3>())};

        Array6d const se3_imu_co{extrinsic.tf.se3_a_b};
        Matrix3d const R_imu_co{geometry::Exp<double>(se3_imu_co.topRows<3>())};

        Vector3d const gravity_imu{R_imu_co * R_w_co.transpose() * extrinsic.gravity.matrix()};

        calibrated_imu_data[timestamp_ns] = imu_data.at(timestamp_ns);
        calibrated_imu_data.at(timestamp_ns).linear_acceleration -= gravity_imu;
    }

    return calibrated_imu_data;
}

void ExtrinsicOptimization::Save(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data, SqlitePtr const db) const {
    auto const [optimized_spline, optimized_extrinsic]{data};

    // TODO(Jack): Hardcoding the imu error to be saved under the entity_id of "frame_a" is a hacky hardcode! What we
    // want here is to make sure that these are saved under the entity_id of the imu which just so happens to be frame_a
    // but that might change! We also do this above with the camera stuff!
    std::string const imu_name{optimized_extrinsic.tf.frame_a};
    std::string const camera_name{optimized_extrinsic.tf.frame_b};

    // ERROR(Jack): This is also unique here that we are inserting extra steps here in the save method! I do not think
    // these steps and their outputs will get removed if the main step entity cache busts. This is a danger!
    // TODO(Jack): Shoudl we automatically delete the non direct dependent rows if we run into this step here? It is a
    // manual clean up but its better than nothing right?
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
    database::InsertStep(db, imu_name, StepType(), HashInputs());
    database::InsertImuErrors(db, imu_name, StepType(), imu_error);

    ImuMeasurements const calibrated_imu_data{CalibrateImuData(data, imu_data_)};
    database::InsertEntity(db, "/imu0/calibrated", Entity::Imu);
    database::InsertImuData(db, "/imu0/calibrated", calibrated_imu_data);
}

}  // namespace reprojection::steps
