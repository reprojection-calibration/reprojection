#include "steps/stereo_rig_init.hpp"

#include <spdlog/fmt/bundled/ranges.h>

#include <ranges>

#include "calibration/init_methods.hpp"
#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

StereoRigInit::StereoRigInit(AssetId const cam0_id, uint64_t const approx_sync_delta_ns,
                             std::vector<CamStageIds> const& cams, SqlitePtr db)
    : cam0_id_{cam0_id}, approx_sync_delta_ns_{approx_sync_delta_ns} {
    for (auto const& cam_i : cams) {
        cam_frames_.emplace(cam_i.asset_id,
                            database::CameraPosesSelect(db.get(), cam_i.bundle_adjustment_id, cam_i.asset_id));
    }
}

Hash StereoRigInit::CacheKey() const {
    Hash const initial_hash{hashing::HashArgs(approx_sync_delta_ns_)};

    // Only hash the frame pose values so we are not dependent on the camera asset ids.
    return std::ranges::fold_left(cam_frames_ | std::views::values, initial_hash,
                                  [](Hash const& hash, auto const& frames) { return hashing::HashArgs(hash, frames); });
}

void StereoRigInit::Execute(StepId step_id, SqlitePtr db) const {
    // NOTE(Jack): Iterate over the the cam0 paired with all cameras. Remember we are working with a tree here not a
    // chain! Also don't forget that when cam0 pais with itself it better produce the identity transform! This is a
    // little redundant but it then eliminates the need for special logic around the reference camera case.
    std::vector<Extrinsic> log_data;
    for (auto const& cam_a_id : cam_frames_ | std::views::keys) {
        Array6d const se3_a_b{calibration::StereoExtrinsicInit(cam_frames_.at(cam_a_id), cam_frames_.at(cam0_id_),
                                                               approx_sync_delta_ns_)};
        Extrinsic const extrinsic_a_b{cam_a_id, cam0_id_, se3_a_b};

        database::ExtrinsicInsert(db.get(), step_id, extrinsic_a_b);
        log_data.push_back(extrinsic_a_b);
    }

    // TODO(Jack): Should we be writing the reprojection errors here? I think that is a really good idea.

    log->info("{{{}, 'result': [{}]}}", StepLogInfo{Type(), step_id}, fmt::join(log_data, ", "));
}

}  // namespace reprojection::steps
