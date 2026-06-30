#include "steps/target_info.hpp"

#include <toml++/toml.h>

#include "config/config_parse.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

std::string TargetInfoStep::HashInputs() const {
    std::ostringstream oss;
    oss << target_config_;

    return hashing::HashArguments(oss.str(), sensor_name_);
}

TargetInfo TargetInfoStep::Compute() const {
    // TODO(Jack): Here we see that the intermediate config::Config::Target type is little redundant because it is
    // basically exactly the TargetInfo type. We should unify these into one single type.
    auto const cfg{config::Config::Target::Parse(*target_config_.as_table())};

    return {cfg.target_type, cfg.size[0], cfg.size[1], cfg.unit_dimension, cfg.asymmetric};
}

TargetInfo TargetInfoStep::Load(SqlitePtr const db) const {
    auto const target_info{database::ReadTargetInfo(db, EntityId())};

    if (not target_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");  // LCOV_EXCL_LINE
    }

    return *target_info;
}

void TargetInfoStep::Save(TargetInfo const& target_info, SqlitePtr const db) const {
    database::InsertTargetInfo(db, EntityId(), target_info);
}

}  // namespace reprojection::steps
