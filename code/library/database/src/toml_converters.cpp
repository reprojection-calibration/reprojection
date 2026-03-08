#include "toml_converters.hpp"

#include <toml++/toml.hpp>

namespace reprojection::database {

// TODO(Jack): Add error handling if for example the length of intrinsics does not match the length expected for that
//  camera model.
std::string ToToml(CameraModel const type, ArrayXd const& intrinsics) {
    toml::table tbl;
    if (type == CameraModel::DoubleSphere) {
        tbl = toml::table{{"fx", intrinsics[0]}, {"fy", intrinsics[1]}, {"cx", intrinsics[2]},
                          {"cy", intrinsics[3]}, {"xi", intrinsics[4]}, {"alpha", intrinsics[5]}};
    } else if (type == CameraModel::Pinhole) {
        tbl = toml::table{{"fx", intrinsics[0]}, {"fy", intrinsics[1]}, {"cx", intrinsics[2]}, {"cy", intrinsics[3]}};
    } else {
        throw std::runtime_error("Implement ToToml(CameraModel) for other camera models!");
    }

    std::ostringstream oss;
    oss << tbl;  // default formatter outputs TOML
    return oss.str();
}

ArrayXd FromToml(CameraModel const type, std::string const& toml_str) {
    if (type == CameraModel::DoubleSphere) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(6);
        intrinsics[0] = tbl["fx"].value<double>().value();
        intrinsics[1] = tbl["fy"].value<double>().value();
        intrinsics[2] = tbl["cx"].value<double>().value();
        intrinsics[3] = tbl["cy"].value<double>().value();
        intrinsics[4] = tbl["xi"].value<double>().value();
        intrinsics[5] = tbl["alpha"].value<double>().value();
        return intrinsics;
    } else if (type == CameraModel::Pinhole) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(4);
        intrinsics[0] = tbl["fx"].value<double>().value();
        intrinsics[1] = tbl["fy"].value<double>().value();
        intrinsics[2] = tbl["cx"].value<double>().value();
        intrinsics[3] = tbl["cy"].value<double>().value();
        return intrinsics;
    } else {
        throw std::runtime_error("Implement FromToml(CameraModel) for other camera models!");
    }
}

}  // namespace reprojection::database