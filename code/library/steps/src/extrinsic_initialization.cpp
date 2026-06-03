#include "steps/extrinsic_initialization.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

std::string ExtrinsicInitialization::CacheKey() const { return caching::CacheKey(sensor_name, imu_data, spline); }

std::pair<Array6d, Array3d> ExtrinsicInitialization::Compute() const {
    auto const [rotation_result, gravity_w]{
        calibration::EstimateCameraImuRotationAndGravity({spline.So3(), spline.GetTimeHandler()}, imu_data)};

    // TODO(Jack): Should we do something with the diagnostics? There are several places now where we ignore the
    // returned optimization diagnostics but I am sure that a user would appreciate these in the database.
    auto const [aa_imu_co, _]{rotation_result};

    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};

    return {tf_imu_co, gravity_w};
}

std::pair<Array6d, Array3d> ExtrinsicInitialization::Load(SqlitePtr const db) const {
    auto const tf_co_imu{database::ReadExtrinsics(db, CalibrationStep::ExtrinsicInitialization, SensorName())};
    auto const gravity_w{database::ReadGravity(db, CalibrationStep::ExtrinsicInitialization, SensorName())};

    if (not tf_co_imu or not gravity_w) {
        std::cout << "WE NEED AN ERROR STRATEGY! ExtrinsicInitialization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    return {*tf_co_imu, *gravity_w};
}

void ExtrinsicInitialization::Save(std::pair<Array6d, Array3d> const& extrinsic, SqlitePtr const db) const {
    auto const [tf_co_imu, gravity_w]{extrinsic};

    database::WriteExtrinsicToDb(tf_co_imu, step_type, sensor_name, db);
    database::WriteGravityToDb(gravity_w, step_type, sensor_name, db);
}

}  // namespace reprojection::steps
