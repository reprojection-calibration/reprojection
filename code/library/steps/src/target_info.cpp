#include "steps/target_info.hpp"

#include <toml++/toml.h>

#include "caching/hashing.hpp"
#include "config/config_parsing.hpp"
#include "config/config_validation.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

std::string TargetInfoStep::CacheKey() const {
    std::ostringstream oss;
    oss << target_config;
    oss << sensor_name;

    return caching::HashArguments(oss.str());
}

TargetInfo TargetInfoStep::Compute() const {
    TargetInfo const target_info{config::ParseTargetConfig(*target_config.as_table())};

    return target_info;
}

TargetInfo TargetInfoStep::Load(SqlitePtr const db) const {
    auto const target_info{database::ReadTargetInfo(db, SensorName())};

    if (not target_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");  // LCOV_EXCL_LINE
    }

    return *target_info;
}

void TargetInfoStep::Save(TargetInfo const& target_info, SqlitePtr const db) const {
    database::InsertTargetInfo(db, SensorName(), target_info);
}

}  // namespace reprojection::steps
