#include "toml_converters.hpp"

namespace reprojection::database {

// TODO(Jack): Add error handling if for example the length of intrinsics does not match the length expected for that
//  camera model.
// TODO(Jack): This makes the assumption that every single intrinsic is in the pinhole family of intrinsics. If we
// one day use some crazy camera model that does not fit that assumption then we need to change this.
std::string ToToml(CameraModel const type, ArrayXd const& intrinsics) {
    toml::table tbl{{"fx", intrinsics[0]}, {"fy", intrinsics[1]}, {"cx", intrinsics[2]}, {"cy", intrinsics[3]}};
    if (type == CameraModel::DoubleSphere) {
        tbl.insert("xi", intrinsics[4]);
        tbl.insert("alpha", intrinsics[5]);
    } else if (type == CameraModel::Pinhole) {
        // Do nothing because tbl is already initialized with pinhole intrinsics - but keep here so we can still count
        // on the else block below to throw if we have an unaccounted for camera model.
    } else if (type == CameraModel::PinholeRadtan4) {
        tbl.insert("k1", intrinsics[4]);
        tbl.insert("k2", intrinsics[5]);
        tbl.insert("p1", intrinsics[6]);
        tbl.insert("p2", intrinsics[7]);
    } else if (type == CameraModel::UnifiedCameraModel) {
        tbl.insert("xi", intrinsics[4]);
    } else {
        throw std::runtime_error("Implement ToToml(CameraModel) for other camera models!");  // LCOV_EXCL_LINE
    }

    std::ostringstream oss;
    oss << tbl;  // Default table formatter outputs toml :)
    return oss.str();
}

ArrayXd FromToml(CameraModel const type, std::string const& toml_str) {
    if (type == CameraModel::DoubleSphere) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(6);

        intrinsics = ReadPinholeValues(tbl, intrinsics);
        intrinsics[4] = tbl["xi"].value<double>().value();
        intrinsics[5] = tbl["alpha"].value<double>().value();

        return intrinsics;
    } else if (type == CameraModel::Pinhole) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(4);

        intrinsics = ReadPinholeValues(tbl, intrinsics);

        return intrinsics;
    } else if (type == CameraModel::PinholeRadtan4) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(8);

        intrinsics = ReadPinholeValues(tbl, intrinsics);
        intrinsics[4] = tbl["k1"].value<double>().value();
        intrinsics[5] = tbl["k2"].value<double>().value();
        intrinsics[6] = tbl["p1"].value<double>().value();
        intrinsics[7] = tbl["p2"].value<double>().value();

        return intrinsics;
    } else if (type == CameraModel::UnifiedCameraModel) {
        auto tbl = toml::parse(toml_str);
        ArrayXd intrinsics(5);

        intrinsics = ReadPinholeValues(tbl, intrinsics);
        intrinsics[4] = tbl["xi"].value<double>().value();

        return intrinsics;
    } else {
        throw std::runtime_error("Implement FromToml(CameraModel) for other camera models!");  // LCOV_EXCL_LINE
    }
}

// WARN(Jack): This function does no error/bounds checking or handling! It is a simple method but in the wrong hands it
// is ticking timebomb if used inappropriately.
ArrayXd ReadPinholeValues(toml::parse_result const& tbl, ArrayXd intrinsics) {
    intrinsics[0] = tbl["fx"].value<double>().value();
    intrinsics[1] = tbl["fy"].value<double>().value();
    intrinsics[2] = tbl["cx"].value<double>().value();
    intrinsics[3] = tbl["cy"].value<double>().value();

    return intrinsics;
}

}  // namespace reprojection::database