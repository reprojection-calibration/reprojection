#include "json_converters.hpp"

#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

namespace reprojection::database {
// TODO(Jack): Add error handling if for example the length of intrinsics does not match the length expected for that
//  camera model.
// TODO(Jack): Add other camera types!
std::string ToJson(CameraModel const type, ArrayXd const& intrinsics) {
    toml::table tbl;
    if (type == CameraModel::DoubleSphere) {
        tbl = toml::table{{"fx", intrinsics[0]}, {"fy", intrinsics[1]}, {"cx", intrinsics[2]},
                          {"cy", intrinsics[3]}, {"xi", intrinsics[4]}, {"alpha", intrinsics[5]}};
    } else {
        throw std::runtime_error("Implement the ToJson method for other types!");
    }

    std::ostringstream oss;
    oss << toml::json_formatter{tbl};

    return oss.str();
}

// WARN JUS TCOPY AND PASTED
ArrayXd FromJson(CameraModel const type, std::string const& json_str) {
    if (type == CameraModel::DoubleSphere) {
        auto j = nlohmann::json::parse(json_str);
        ArrayXd intrinsics(6);
        intrinsics[0] = j["fx"].get<double>();
        intrinsics[1] = j["fy"].get<double>();
        intrinsics[2] = j["cx"].get<double>();
        intrinsics[3] = j["cy"].get<double>();
        intrinsics[4] = j["xi"].get<double>();
        intrinsics[5] = j["alpha"].get<double>();
        return intrinsics;
    } else {
        throw std::runtime_error("Implement FromJson for other types!");
    }
}
}  // namespace reprojection::database