
#include "caching/serialize.hpp"

#include "types/enums.hpp"

namespace reprojection::caching {

std::string Serialize(CameraInfo const& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss << data.sensor_name << "|";
    oss << ToString(data.camera_model) << "|";
    oss << data.bounds.u_min << "," << data.bounds.u_max << "," << data.bounds.v_min << "," << data.bounds.v_max << "|";

    return oss.str();
}

std::string Serialize(CameraMeasurements const& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    for (auto const& [timestamp_ns, target] : data) {
        oss << timestamp_ns << "|";

        MatrixX2d const& pixels{target.bundle.pixels};
        for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
            oss << pixels.row(i)(0) << "," << pixels.row(i)(1) << ";";
        }
        oss << "|";
        MatrixX3d const& points{target.bundle.points};
        for (Eigen::Index i{0}; i < points.rows(); ++i) {
            oss << points.row(i)(0) << "," << points.row(i)(1) << "," << points.row(i)(2) << ";";
        }
        oss << "|";
        ArrayX2i const& indices{target.indices};
        for (Eigen::Index i{0}; i < indices.rows(); ++i) {
            oss << indices.row(i)(0) << "," << indices.row(i)(1) << ";";
        }
        oss << "|";
    }

    return oss.str();
}

}  // namespace reprojection::caching
