
#include "serialize.hpp"

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

        SerializeEigenByRows(target.bundle.pixels, oss);
        oss << "|";
        SerializeEigenByRows(target.bundle.points, oss);
        oss << "|";
        SerializeEigenByRows(target.indices, oss);
        oss << "|";
    }

    return oss.str();
}

std::string Serialize(CameraState const& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    SerializeEigenByRows(data.intrinsics, oss);
    oss << "|";

    return oss.str();
}

std::string Serialize(Frames const& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    for (auto const& [timestamp_ns, frame] : data) {
        oss << timestamp_ns << "|";
        SerializeEigenByRows(frame.pose, oss);
        oss << "|";
    }

    return oss.str();
}

}  // namespace reprojection::caching
