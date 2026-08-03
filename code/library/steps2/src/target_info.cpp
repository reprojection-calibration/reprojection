#include "steps/target_info.hpp"

#include "hashing/hashing.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

TargetInfoStep::TargetInfoStep(AssetId target_id, config::Config::Target const& target)
    : target_id_{target_id}, target_{target} {}

Hash TargetInfoStep::CacheKey() const { return hashing::HashArguments(target_id_.value, target_); }

void TargetInfoStep::Execute(StepId step_id, database::CalibrationDatabase& db) const {
    TargetInfo const target_info{target_.target_type, target_.size[0], target_.size[1], target_.unit_dimension,
                                 target_.asymmetric};

    log->info(
        "{{'step_id': '{}', 'asset_id': '{}', 'target_info': {{'target_type': {}, 'rows': {}, 'cols': {}, "
        "'unit_dimension': {}, 'asymmetric': {}}}}}}}",
        step_id.value, target_id_.value, ToString(target_info.target_type), target_info.height, target_info.width,
        target_info.unit_dimension, target_info.asymmetric);

    db.TargetInfoInsert(step_id, target_id_, target_info);
}

}  // namespace reprojection::steps
