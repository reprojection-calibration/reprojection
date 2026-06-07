#include "steps/extrinsic_initialization.hpp"

#include "hashing/hashing.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_imu_calibration.hpp"

namespace reprojection::steps {

std::string ExtrinsicInitialization::CacheKey() const {
    return hashing::HashArguments(extrinsic_id, imu_name, imu_data, spline.ControlPoints(), spline.GetTimeHandler().t0_ns_,
                             spline.GetTimeHandler().delta_t_ns_);
}

std::pair<Array6d, Array3d> ExtrinsicInitialization::Compute() const {
    auto const [rotation_result,
                gravity_w]{calibration::EstimateCameraImuAlignment({spline.So3(), spline.GetTimeHandler()}, imu_data)};

    // TODO(Jack): Should we do something with the diagnostics? There are several places now where we ignore the
    // returned optimization diagnostics but I am sure that a user would appreciate these in the database.
    auto const [aa_imu_co, _]{rotation_result};

    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};

    return {tf_imu_co, gravity_w};
}

std::pair<Array6d, Array3d> ExtrinsicInitialization::Load(SqlitePtr const db) const {
    auto const tf_imu_co{database::ReadExtrinsics(db, SensorName(), CalibrationStep::ExtrinsicInitialization)};
    auto const gravity_w{database::ReadGravity(db, SensorName(), CalibrationStep::ExtrinsicInitialization)};

    if (not tf_imu_co or not gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicInitialization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    return {*tf_imu_co, *gravity_w};
}

void ExtrinsicInitialization::Save(std::pair<Array6d, Array3d> const& extrinsic, SqlitePtr const db) const {
    auto const [tf_imu_co, gravity_w]{extrinsic};

    database::InsertExtrinsic(db, SensorName(), step_type, tf_imu_co);
    database::InsertGravity(db, SensorName(), step_type, gravity_w);

    // TODO(Jack): We save the imu errors here under the imu and not the extrinsic identity name! Is it hacky here that
    // we use a second sensor name and also write an additional step to the database outside of the sanctioned step
    // runner workflow?
    ImuErrors const error{optimization::EvaluateImuError(imu_data, tf_imu_co, gravity_w, spline)};
    database::InsertStep(db, imu_name, step_type, CacheKey());
    database::InsertImuErrors(db, imu_name, step_type, error);
}

}  // namespace reprojection::steps
