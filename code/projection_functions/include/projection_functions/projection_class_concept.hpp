#include <type_traits>

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

template <typename T>
concept HasIntrinsicsSize = requires {
    T::Size;
    std::is_integral_v<T>;
};

template <typename T>
concept CanProject = requires(Eigen::Array<double, T::Size, 1> const& intrinsics, Array3d const& p_co) {
    // NOTE(Jack): We need to manually check the requirement parameters because Eigen does not play nice with concepts
    // (see https://stackoverflow.com/questions/79804552/c-concept-constraint-requirement-is-not-strict). It maybe has
    // something to do with the eigen not being "SFINAE-friendly" (?).
    { intrinsics } -> std::same_as<Eigen::Array<double, T::Size, 1> const&>;
    { p_co } -> std::same_as<Array3d const&>;

    { T::template Project<double>(intrinsics, p_co) } -> std::same_as<Array2d>;
};

template <typename T>
concept CanUnproject = requires(Eigen::Array<double, T::Size, 1> const& intrinsics, Array2d const& pixel) {
    { intrinsics } -> std::same_as<Eigen::Array<double, T::Size, 1> const&>;
    { pixel } -> std::same_as<Array2d const&>;

    { T::Unproject(intrinsics, pixel) } -> std::same_as<Array3d>;
};

/**
 * \brief Defines what we call the "projection class" interface, which are static classes that implement our camera
 * models and satisfy the HasIntrinsicsSize, CanProject, and CanUnproject concepts.
 *
 * We need the ability to enforce an interface on our camera projection classes without introducing virtual functions.
 * I did not benchmark it, so technically it is speculation, but the idea of using virtual functions in our core
 * optimization seems a little careless. The ProjectionClass concepts provides a way for consumers of the projection
 * classes to enforce an interface without resorting to a pure virtual base class.
 *
 * For an example of how this concept is used see the implementation of Camera_T. Note that technically it is not
 * strictly necessary to use a concept here. We can remove it there and the code will still compile. But I wanted to
 * make it as clear as possible to future maintainers adding new projection classes, what is required of them.
 */
template <typename T>
concept ProjectionClass = HasIntrinsicsSize<T> and CanProject<T> and CanUnproject<T>;

}  // namespace reprojection::projection_functions