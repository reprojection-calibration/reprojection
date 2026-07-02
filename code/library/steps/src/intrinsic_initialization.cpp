#include "steps/intrinsic_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

std::string IntrinsicInitialization::HashInputs() const { return hashing::HashArguments(camera_info_, targets_); }

CameraState IntrinsicInitialization::Compute() const {
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info_.camera_model, camera_info_.bounds.v_max,
                                                            camera_info_.bounds.u_max, targets_, num_threads_)};

    if (not intrinsics.has_value()) {
        throw std::runtime_error                                                                       // LCOV_EXCL_LINE
            ("We have no error handling strategy for failed IntrinsicInitializationStep::Compute()");  // LCOV_EXCL_LINE
    }

    return CameraState{*intrinsics};
}

CameraState IntrinsicInitialization::Load(SqlitePtr const db) const {
    auto const loaded_intrinsics{database::ReadIntrinsics(db, EntityId(), StepType(), camera_info_.camera_model)};

    if (not loaded_intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Load()");  // LCOV_EXCL_LINE
    }

    return CameraState{*loaded_intrinsics};
}

void IntrinsicInitialization::Save(CameraState const& intrinsics, SqlitePtr const db) const {
    database::InsertIntrinsics(db, EntityId(), StepType(), camera_info_.camera_model, intrinsics);
}

}  // namespace reprojection::steps
