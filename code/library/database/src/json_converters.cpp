#include "json_converters.hpp"

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
        auto tbl = toml::parse(json_str);  // toml++ can parse JSON
        ArrayXd intrinsics(6);
        intrinsics[0] = tbl["fx"].value<double>().value();
        intrinsics[1] = tbl["fy"].value<double>().value();
        intrinsics[2] = tbl["cx"].value<double>().value();
        intrinsics[3] = tbl["cy"].value<double>().value();
        intrinsics[4] = tbl["xi"].value<double>().value();
        intrinsics[5] = tbl["alpha"].value<double>().value();
        return intrinsics;
    } else {
        throw std::runtime_error("Implement FromJson for other types!");
    }
}

}  // namespace reprojection::database