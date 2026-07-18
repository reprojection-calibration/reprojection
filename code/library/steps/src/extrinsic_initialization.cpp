#include "steps/extrinsic_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "optimization/extrinsic_optimization.hpp"

namespace reprojection::steps {

std::string ExtrinsicInitialization::HashInputs() const {
    return hashing::HashArguments(imu_name_, camera_name_, imu_data_, spline_.ControlPoints(),
                                  spline_.GetTimeHandler().t0_ns_, spline_.GetTimeHandler().delta_t_ns_);
}

ImuCamExtrinsic ExtrinsicInitialization::Compute() const {
    auto const [rotation_result, gravity_w]{calibration::EstimateCameraImuAlignment(spline_, imu_data_, num_threads_)};

    // TODO(Jack): Should we do something with the diagnostics? There are several places now where we ignore the
    // returned optimization diagnostics but I am sure that a user would appreciate these in the database.
    auto const [aa_imu_co, _]{rotation_result};

    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};

    return {Extrinsic{imu_name_, camera_name_, tf_imu_co}, gravity_w};
}

ImuCamExtrinsic ExtrinsicInitialization::Load(SqlitePtr const db) const {
    auto const tf_imu_co{database::ReadExtrinsics(db, EntityId(), StepType())};
    auto const gravity_w{database::ReadGravity(db, EntityId(), StepType())};

    if (not tf_imu_co or not gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicInitialization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    return {*tf_imu_co, *gravity_w};
}

void ExtrinsicInitialization::Save(ImuCamExtrinsic const& extrinsic, SqlitePtr const db) const {
    database::InsertExtrinsic(db, EntityId(), StepType(), extrinsic.tf);
    database::InsertGravity(db, EntityId(), StepType(), extrinsic.gravity);

    // TODO(Jack): We save the imu errors here under the imu and not the extrinsic identity name! Is it hacky here that
    // we use a second sensor name and also write an additional step to the database outside of the sanctioned step
    // runner workflow?
    ImuErrors const error{optimization::EvaluateImuError(imu_data_, extrinsic, spline_)};
    database::InsertImuErrors(db, EntityId(), StepType(), imu_name_, error);
}

}  // namespace reprojection::steps
