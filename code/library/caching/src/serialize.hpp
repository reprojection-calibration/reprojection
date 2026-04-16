
#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string Serialize(std::string_view data);

// TEST
// TEST
// TEST
std::string Serialize(EncodedImages const& data);

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

std::string Serialize(CameraState const& data);

std::string Serialize(Frames const& data);

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
