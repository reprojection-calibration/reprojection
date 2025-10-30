#include <type_traits>

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

template <typename T>
concept HasIntrinsicsSize = requires {
    T::Size;
    std::is_integral_v<T>;
};

template <typename T>
concept CanProject = requires(Eigen::ArrayXd const& intrinsics, Array3d const& point) {
    // NOTE(Jack): We need to manually check the requirement parameters because Eigen does not play nice with concepts
    // (see https://stackoverflow.com/questions/79804552/c-concept-constraint-requirement-is-not-strict). It has
    // something to do with the eigen not being "SFINAE-friendly".
    { intrinsics } -> std::same_as<Eigen::ArrayXd const&>;
    { point } -> std::same_as<Array3d const&>;

    { T::template Project<double>(intrinsics, point) } -> std::same_as<Array2d>;
};

template <typename T>
concept CanUnproject = requires(Eigen::ArrayXd const& intrinsics, Array2d const& pixel) {
    { intrinsics } -> std::same_as<Eigen::ArrayXd const&>;
    { pixel } -> std::same_as<Array2d const&>;

    { T::Unproject(intrinsics, pixel) } -> std::same_as<Array3d>;
};

template <typename T>
concept ProjectionClass = HasIntrinsicsSize<T> and CanProject<T> and CanUnproject<T>;

}  // namespace reprojection::projection_functions