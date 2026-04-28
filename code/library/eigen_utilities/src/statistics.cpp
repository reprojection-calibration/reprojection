#include "eigen_utilities/statistics.hpp"

namespace reprojection::eigen_utilities {

double Median(ArrayXd data) {
    std::sort(data.begin(), data.end());

    return data.size() % 2 == 0 ? data.segment((data.size() - 2) / 2, 2).mean() : data(data.size() / 2);
}

}  // namespace reprojection::eigen_utilities