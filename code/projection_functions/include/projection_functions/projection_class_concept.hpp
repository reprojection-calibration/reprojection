#include <type_traits>

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \defgroup projection_class Projection Classes
 * \brief The lowest level abstraction for defining camera model projection functions.
 *
 * These are the only places where the projection math is actually implemented, and are used in one form or another
 * throughout the entire code base, for both optimization and non-optimization related code. A projection class consists
 * of three parts; (1) the intrinsic calibration size (ex. Pinhole::Size=4 [fx, fy, cx, cy]), (2) a Project() function
 * and (3) an Unproject() function.
 *
 * The size parameter defines the length of the intrinsics array used by the camera model. For example the pinhole
 * camera model has a size of four and the double sphere has a size of six [fx, fy, cx, cy, xi, alpha]. For all models
 * the pinhole parameters should be the first four parameters in the array and use the same order used by the pinhole
 * model. This helps make some code reuse easier and the intrinsic order standard across all models. For example if
 * you look in the double sphere model you can see in both the project and uproject function how the pinhole models
 * project and unproject models are applied. This eliminates code/logic duplication.
 *
 * One open problem is that there is currently no model or interface which enforces a certain intrinsic parameter order
 * or usage. Given the many contexts and places we want to use the projection classes this is no trivial. The size
 * parameter is one step in the right direction, but maybe not the last one. One idea to consider would be defining
 * structs like struct PinholeIntrinsics {double fx; double fy; etc.}; and then composing more compliated models
 * intrinsics like struct DoubleSphereIntrinsics {PinholeIntrinsics pinhole; double xi; double alpha;};
 *
 *
 */

template <typename T>
concept HasIntrinsicsSize = requires {
    T::Size;
    std::is_integral_v<T>;
};

// TODO(Jack): Should we check HasIntrinsicSize before we do the requires section here? Because there we od T::Size
// without actually confirming that we have size which might cause errors. If we only use these in the ProjectionClass
// concept then I think it is no problem because we check HasIntrinsicSize size first, but if we use them individually
// the topic might come up again.
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