#include "steps/cam_cam_extrinsic_init.hpp"

#include <spdlog/fmt/bundled/ranges.h>

#include <ranges>

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

StereoRigInit::StereoRigInit(std::vector<CamStageIds> const& camera_calibrations,
                                         uint64_t const approx_sync_delta_ns, SqlitePtr db)
    : approx_sync_delta_ns_{approx_sync_delta_ns} {
    for (auto const& camera : camera_calibrations) {
        cam_ids_.emplace_back(camera.camera_id);
        camera_frames_.emplace(camera.camera_id,
                               database::CameraPosesSelect(db.get(), camera.bundle_adjustment_id, camera.camera_id));
    }
}

Hash StereoRigInit::CacheKey() const {
    Hash const initial_hash{hashing::HashArguments(approx_sync_delta_ns_)};

    // Only hash the frame pose values so we are not dependent on the camera asset ids.
    return std::ranges::fold_left(
        camera_frames_ | std::views::values, initial_hash,
        [](Hash const& hash, auto const& frames) { return hashing::HashArguments(hash, frames); });
}

void StereoRigInit::Execute(StepId step_id, SqlitePtr db) const {
    // Iterate over the the first camera paired with every other camera. Remember we are working with a tree here not a
    // chain!
    std::vector<Extrinsic> log_data;
    for (auto const& camera_a_id : cam_ids_ | std::views::drop(1)) {
        // TODO(Jack): Formalize this "first cam is always reference" logic!
        auto const& camera_b_id{cam_ids_.front()};

        // TODO(Jack): Set the sync tolerance from a config!
        // ERROR(Jack): 1'000'000ns is a very tight tolerance!
        Array6d const se3_a_b{calibration::InitializeCamCamExtrinsic(
            camera_frames_.at(camera_a_id), camera_frames_.at(camera_b_id), approx_sync_delta_ns_)};
        Extrinsic const extrinsic_a_b{camera_a_id, camera_b_id, se3_a_b};

        database::ExtrinsicInsert(db.get(), step_id, extrinsic_a_b);
        log_data.push_back(extrinsic_a_b);
    }

    // TODO(Jack): Should we be writing the reprojection errors here? I think that is a really good idea.

    log->info("{{{}, 'result': [{}]}}", StepLogInfo{Type(), step_id}, fmt::join(log_data, ", "));
}

}  // namespace reprojection::steps
