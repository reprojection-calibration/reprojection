#pragma once

#include <string>

#include "config/config_parse.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

// TODO(Jack): I would like these to be private to the module here but the fact is that HashArguments() needs access to
// these functions, and therefore they need to be part of the modules public interface. Is that right?

namespace reprojection::hashing {

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

std::string Serialize(CameraModel const data);

std::string Serialize(Intrinsic const& data);

// TODO(Jack): Test!
std::string Serialize(EncodedImages const& data);

std::string Serialize(Extrinsic const& data);

std::string Serialize(Frames const& data);

std::string Serialize(ImuMeasurements const& data);

std::string Serialize(OptimizationState const& data);

std::string Serialize(TargetInfo const& data);

std::string Serialize(config::Config::Target const& data);

// TODO(Jack): Can we write one single generic method for all the types that have a default std::to_string method?
std::string Serialize(std::string_view data);

std::string Serialize(std::vector<AssetId> const& data);

template <typename T>
concept Stringifiable = requires(T const value) {
    { std::to_string(value) } -> std::same_as<std::string>;
};

template <typename T>
    requires Stringifiable<T>
std::string Serialize(T const data) {
    return std::to_string(data);
}

template <typename Derived>
std::string Serialize(Eigen::DenseBase<Derived> const& m) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    for (Eigen::Index i = 0; i < m.rows(); ++i) {
        for (Eigen::Index j = 0; j < m.cols(); ++j) {
            if (j > 0) {
                oss << ",";
            }
            oss << m(i, j);
        }
        oss << ";";
    }

    return oss.str();
}

}  // namespace reprojection::hashing
