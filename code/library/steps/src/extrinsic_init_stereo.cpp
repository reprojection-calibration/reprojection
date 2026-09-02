#include "steps/extrinsic_init_stereo.hpp"

#include "calibration/calibration_utils.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

// TODO TEST!!!

ExtrinsicInitStereo::ExtrinsicInitStereo(AssetId const camera_a_id, StepId const camera_a_step,
                                         AssetId const camera_b_id, StepId const camera_b_step, SqlitePtr const db)
    : camera_a_id_{camera_a_id},
      camera_a_step_{camera_a_step},
      frames_a_{database::CameraPosesSelect(db.get(), camera_a_step, camera_a_id)},
      camera_b_id_{camera_b_id},
      camera_b_step_{camera_b_step},
      frames_b_{database::CameraPosesSelect(db.get(), camera_b_step, camera_b_id)} {}

Hash ExtrinsicInitStereo::CacheKey() const { return hashing::HashArguments(frames_a_, frames_b_); }

void ExtrinsicInitStereo::Execute(StepId const step_id, SqlitePtr const db) const {
    // TODO(Jack): Check that the frame order matches what we actually want!

    // MAKE THE SYNC TOLERANCE AN APPLICATION CONFIG AND PASS IT HERE! Make part of the cache key too.
    // TODO(Jack): We will need to resync the frames when we do the full bundle adjustment. There is nothing wrong with
    // that as long as we are sure the time tolerance is  the same in both places. It is better to just recalculate it
    // from the raw data than storing a synchronized set extra just for that purpose.
    auto const [frames_a_synced, frames_b_synced]{calibration::SynchronizeFrames(frames_a_, frames_b_, 1000)};

    Array6d const tf_a_b{calibration::InitializeCamCamExtrinsic(frames_a_, frames_b_)};
    Extrinsic const extrinsic{camera_a_id_, camera_b_id_, tf_a_b};

    database::ExtrinsicInsert(db.get(), step_id, extrinsic);
}

}  // namespace reprojection::steps
