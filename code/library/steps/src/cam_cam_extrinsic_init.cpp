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

CamCamExtrinsicInit::CamCamExtrinsicInit(std::vector<CameraCalibration> const& camera_calibrations, SqlitePtr db) {
    // Extract only the parts we actually need.
    std::ranges::transform(camera_calibrations, std::back_inserter(states_),
                           [](auto const& calibration) { return CamCamExtrinsicInitState{calibration}; });

    for (auto const& state : states_) {
        Frames const frames{database::CameraPosesSelect(db.get(), state.frames_id, state.camera_id)};
        camera_frames_.insert({state.camera_id, frames});
    }
}

Hash CamCamExtrinsicInit::CacheKey() const {
    // Only hash the frame pose values so we are not dependent on the camera asset ids.
    return std::ranges::fold_left(
        camera_frames_ | std::views::values, Hash{},
        [](Hash const& hash, auto const& frames) { return hashing::HashArguments(hash, frames); });
}

void CamCamExtrinsicInit::Execute(StepId step_id, SqlitePtr db) const {
    // Iterate over the the first camera paired with every other camera. Remember we are working with a tree here not a
    // chain!
    std::vector<Extrinsic> log_data;
    for (auto const& camera_a : states_ | std::views::drop(1)) {
        // TODO(Jack): Formalize this "first cam is always reference" logic!
        auto const& camera_b{states_.front()};

        // TODO(Jack): Set the sync tolerance from a config!
        // ERROR(Jack): 1'000'000ns is a very tight tolerance!
        Array6d const se3_a_b{calibration::InitializeCamCamExtrinsic(camera_frames_.at(camera_a.camera_id),
                                                                     camera_frames_.at(camera_b.camera_id), 1'000'000)};
        Extrinsic const extrinsic_a_b{camera_a.camera_id, camera_b.camera_id, se3_a_b};

        database::ExtrinsicInsert(db.get(), step_id, extrinsic_a_b);
        log_data.push_back(extrinsic_a_b);
    }

    log->info("{{{}, 'result': [{}]}}", StepLogInfo{Type(), step_id},
              fmt::join(log_data, ", "));
}

}  // namespace reprojection::steps
