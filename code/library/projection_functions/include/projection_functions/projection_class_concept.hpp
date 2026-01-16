#include <optional>
#include <type_traits>

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \defgroup projection_classes Projection Classes
 * \brief The lowest level abstraction for defining camera model projection functions.
 *
 * These are the only places where the projection math is actually implemented, and are used in one form or another
 * throughout the entire code base, for both optimization and non-optimization related code. A projection class consists
 * of three parts; (1) the intrinsic calibration size (ex. Pinhole::Size=4 [fx, fy, cx, cy]), (2) a Project() function
 * and (3) an Unproject() function. Due to the fact that we want to avoid virtual functions and the runtime cost
 * associated with them, we use the ProjectionClass concept to enforce the interface, instead of using a pure virtual
 * parent class.
 *
 * The size parameter defines the length of the intrinsics array used by the camera model. For example the pinhole
 * camera model has a size of four and the double sphere has a size of six [fx, fy, cx, cy, xi, alpha]. For all models
 * the pinhole parameters should be the first four parameters in the array and use the same order used by the pinhole
 * model. This helps make some code reuse easier and the intrinsic order standard across all models. For example if
 * you look in the double sphere model you can see in both the project and uproject function how the pinhole models
 * project and unproject models are applied. This eliminates code/logic duplication.
 *
 * Note that only the size parameter is encoded into a projection class, not the intrinsic array type itself. Each
 * application where the projection classes are used must define its own intrinsic array as needed.
 *
 * One open problem is that there is currently no model or interface which enforces a specific intrinsic parameter order
 * or usage. Given the many contexts and places we want to use the projection classes this is not trivial to do. The
 * size parameter is one step in the right direction, but maybe not the last one. One idea to consider would be defining
 * structs like `struct PinholeIntrinsics {double fx; double fy; etc.};` and then composing more complicated model's
 * intrinsics like `struct DoubleSphereIntrinsics {PinholeIntrinsics pinhole; double xi; double alpha;};`. For now
 * however that is future music.
 *
 * The project function defines the heart of any camera model, projecting points to pixels. The project function expects
 * points to be 3D and in the camera's optical frame (z-forward, x-left, y-down). A value in image pixel space is
 * returned.
 *
 * The unproject function defines how pixels are unprojected to rays in the 3D camera optical frame. The unproject
 * function expects image space pixels as input. Note that the value returned will NOT be the same 3D point that was
 * projected to form the pixel. We are talking about projective transforms here, which means we loose the depth
 * dimension. This dimension cannot be magically recovered in the unproject function, we can only recover the direction.
 * This fact explains why some people describe cameras in the context of 3D reconstruction as "bearing" sensors, able to
 * directly measure and track the direction of a feature, but not its depth.
 */

// TODO(Jack): Make sure the concepts and their documentation are picked up by doxygen.

/**
 * \brief Concept that enforces a type has an integer member named `Size`.
 */
template <typename T>
concept HasIntrinsicsSize = requires {
    T::Size;
    std::is_integral_v<T>;  // WARN(Jack): There are many integral types (ex. char), this condition should be more
                            // strict!
};

// TODO(Jack): Should we check HasIntrinsicSize before we do the requires section here? Because there we do T::Size
// without actually confirming that we have size which might cause errors. If we only use these in the ProjectionClass
// concept then I think it is no problem because we check HasIntrinsicSize size first, but if we use them individually
// the topic might come up again.
/**
 * \brief Concept that enforces a type has a `Project()` method that take an intrinsic array and 3D point and returns a
 * 2D point.
 *
 * The project function, unlike the unproject function, is templated. This is because the function is used in the core
 * optimization where it needs to be able to handle ceres::Jet types to support using ceres::AutoDiffCostFunction. Maybe
 * one day if we implement the jacobians for all project methods then we can hard-code this as a double, but we are not
 * there yet!
 */
template <typename T>
concept CanProject = requires(Eigen::Array<double, T::Size, 1> const& intrinsics, Array3d const& p_co) {
    // NOTE(Jack): We need to manually check the requirement parameters because Eigen does not play nice with concepts
    // (see https://stackoverflow.com/questions/79804552/c-concept-constraint-requirement-is-not-strict). It maybe has
    // something to do with the eigen not being "SFINAE-friendly" (?).
    { intrinsics } -> std::same_as<Eigen::Array<double, T::Size, 1> const&>;
    { p_co } -> std::same_as<Array3d const&>;

    { T::template Project<double>(intrinsics, p_co) } -> std::same_as<std::optional<Array2d>>;
};

/**
 * \brief Concept that enforces a type has an `Unproject()` method that take an intrinsic array and 2D point and returns
 * a 3D point.
 */
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