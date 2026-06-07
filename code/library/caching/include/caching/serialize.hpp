#pragma once

#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

std::string Serialize(CameraModel const data);

std::string Serialize(CameraState const& data);

// WARN(Jack): The spline module does a very poor job of isolating dependencies and exposing types. As I did not want to
// have the caching module depend on the spline module (very different abstraction levels), I instead decided to
// decompose the spline and pass its basic types. This is not nice because instead of using spline::MatrixNXd like we
// should for the C3 spline control points we redefine it and use Matrix3Xd here instead. It would have been nice just
// to be able to pass the C3CubicSpline here directly like we do for all the other calibration types. I think the real
// solution is to refactor the spline stuff into the generic internal types package as this is getting a little out of
// hand.
std::string Serialize(Eigen::Matrix<double, 6, -1> const& data);

// TODO(Jack): Test!
std::string Serialize(EncodedImages const& data);

std::string Serialize(Frames const& data);

std::string Serialize(ImuMeasurements const& data);

std::string Serialize(OptimizationState const& data);

std::string Serialize(TargetInfo const& data);

std::string Serialize(std::string_view data);

std::string Serialize(uint64_t const data);

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

}  // namespace reprojection::caching
