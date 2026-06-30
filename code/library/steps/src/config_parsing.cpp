#include "steps/config_parsing.hpp"

namespace reprojection::steps {

config::Config ConfigParsing(toml::table const& cfg_table) {
    config::Config const cfg{config::Config::Parse(cfg_table)};
}

}  // namespace reprojection::steps
