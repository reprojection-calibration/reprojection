#include "steps/target_info.hpp"

#include "hashing/hashing.hpp"

namespace reprojection::steps {

TargetInfoStep::TargetInfoStep(AssetId target_id, config::Config::Target const& target)
    : target_id_{target_id}, target_{target} {}

Hash TargetInfoStep::CacheKey() const { return hashing::HashArguments(target_id_.value, target_); }

void TargetInfoStep::Execute(StepId step_id, database::CalibrationDatabase& db) const {
    TargetInfo const target_info{target_.target_type, target_.size[0], target_.size[1], target_.unit_dimension,
                                 target_.asymmetric};

    db.TargetInfoInsert(step_id, target_id_, target_info);
}

}  // namespace reprojection::steps
