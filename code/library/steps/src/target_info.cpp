#include "steps/target_info.hpp"

#include <toml++/toml.h>

#include "config/config_parsing.hpp"
#include "config/config_validation.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

std::string TargetInfoStep::HashInputs() const {
    std::ostringstream oss;
    oss << target_config_;
    oss << sensor_name_;

    return hashing::HashArguments(oss.str());
}

TargetInfo TargetInfoStep::Compute() const {
    TargetInfo const target_info{config::ParseTargetConfig(*target_config_.as_table())};

    return target_info;
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
