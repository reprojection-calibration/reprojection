#pragma once

#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

std::string Serialize(CameraModel const data);

std::string Serialize(CameraState const& data);

// TODO(Jack): Test!
std::string Serialize(EncodedImages const& data);

std::string Serialize(Frames const& data);

std::string Serialize(ImuMeasurements const& data);

std::string Serialize(TargetInfo const& data);

std::string Serialize(std::string_view data);

template <typename Derived>
void SerializeEigenByRows(Eigen::DenseBase<Derived> const& m, std::ostream& os) {
    for (Eigen::Index i = 0; i < m.rows(); ++i) {
        for (Eigen::Index j = 0; j < m.cols(); ++j) {
            if (j > 0) {
                os << ",";
            }
            os << m(i, j);
        }
        os << ";";
    }
}

template <typename Derived>
std::string Serialize(Eigen::DenseBase<Derived> const& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    for (Eigen::Index i{0}; i < data.cols(); ++i) {
        SerializeEigenByRows(data.col(i), oss);
        oss << "|";
    }

    return oss.str();
}

}  // namespace reprojection::caching
