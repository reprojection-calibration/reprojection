#include "steps/target_info.hpp"

#include <toml++/toml.h>

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "config/config_validation.hpp"

namespace reprojection::steps {

std::string TargetInfoStep::CacheKey() const {
    std::ostringstream oss;
    oss << target_config;
    oss << sensor_name;

    return caching::CacheKey(oss.str());
}

TargetInfo TargetInfoStep::Compute() const {
    if (auto const error_msg{config::ValidateTargetConfig(target_config)}) {
        throw std::runtime_error(error_msg->msg);
    }

    bool asymmetric{false};
    if (auto const node{target_config.at_path("circle_grid.asymmetric")}) {
        asymmetric = node.as_boolean()->get();
    }

    TargetInfo const target_info{ToTargetType(target_config["type"].as_string()->get()),
                                 static_cast<int>(target_config["pattern_size"].as_array()->at(0).as_integer()->get()),
                                 static_cast<int>(target_config["pattern_size"].as_array()->at(1).as_integer()->get()),
                                 asymmetric};

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
    database::WriteToDb(target_info, SensorName(), db);
}

}  // namespace reprojection::steps
