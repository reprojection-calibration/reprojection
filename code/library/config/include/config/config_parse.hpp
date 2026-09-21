#pragma once

#include <array>
#include <string>
#include <thread>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

// Doc format taken from https://www.doxygen.nl/manual/docblocks.html#cppblock

// TODO(Jack): Make the actual config datatype independent of the parsing methods. It looks nice to have them together
// like this, but I do not think it is required. One downside of the current setup is that this is found in the config::
// namespace which makes using it more verbose. It also has a dependency on the toml package which is then transiently
// forces on anyone that needs the config, even if they just want the type and not the parsing.

struct Config {
    static Config Parse(toml::table const& table);

    struct Application {
        static Application Parse(toml::table const& table);

        /*! \brief Stereo frame approximate time synchronization threshold.
         *
         * A timestamp within ±`approx_sync_delta_ns` is considered synchronized. The default value of 3ms is chosen to
         * represent about 10% of a 30Hz camera's image interval.
         *
         * This value is only used to synchronize stereo image streams and nothing else. Visual-inertial calibration
         * uses a continuous spline representation and therefore does not require synchronization.
         *
         *  \par Config Status
         *  Optional.
         *
         *  \par Config Note
         *  The actual configuration file parameter should be input in milliseconds and named accordingly.
         *  \code{.toml}
         *  approx_sync_delta_ms = 3.0
         *  \endcode
         */
        uint64_t approx_sync_delta_ns{3'000'000};
        /*! \brief Visualize the target extraction step.
         *
         *  \par Config Status
         *  Optional.
         *
         *  \warning A GUI is required to display the visualization. Set to false if running headless.
         */
        bool show_extraction{false};
        /*! \brief Number of threads available.
         *
         * Number of threads used to multithread the Ceres Solver calls. The default behavior automatically detects the
         * number of available threads and uses that value minus one.
         *
         * If your system is not compatible with the C++ std::thread API you must manually set this parameter.
         *
         *  \par Config Status
         *  Optional.
         */
        int threads{std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1)};
    };

    /*! \brief The camera sensor calibration configuration.
     *
     * Every calibration workflow requires at least one configured camera, and each calibrated camera must have its own
     * unique configuration. The first camera (i.e. `cam0`) is taken as the reference camera whose coordinate frame
     * defines the calibration rig for stereo and visual-inertial calibration workflows.
     *
     *  \par Config Status
     *  Required.
     *
     *  \par Config Note
     *  The camera configuration requires an "indexed configuration" format. Each camera configuration table must be
     *  named as `[cam<index>]` where `index` starts from zero and increases incrementally by one for each additional
     *  camera.
     *  \code{.toml}
     *  [cam0]
     *  ...
     *
     *  [cam1]
     *  ...
     *  \endcode
     */
    struct Camera {
        // NOTE(Jack): We have to pass the index here because that comes from the parsed table header which is already
        // removed by the time we parse its' contents.
        static Camera Parse(toml::table const& table, int index);

        /*! \brief The camera's selected projection model type.
         *
         *  \par Config Status
         *  Required.
         *
         *  \par Config Note
         *  The configuration file parameter should be written in snake_case. See enum \ref reprojection::CameraModel
         *  for the list of supported values and the function \ref reprojection::ToCameraModel for the
         *  PascalCase <-> snake_case conversion logic.
         *  \code{.toml}
         *  camera_model = "double_sphere"
         *  \endcode
         */
        CameraModel camera_model;
        /*! \brief Estimated focal length (in pixel).
         *
         * Used to initialize the robust intrinsic initialization strategy. The actual focal length should be within one
         * half the provided `focal_length` parameter.
         *
         * The default behavior when no `focal_length` configuration file parameter is passed sets the initial focal
         * length to one half the maximum image dimension. If you believe your focal length is larger than the largest
         * image dimension, you probably want to set this parameter.
         *
         *  \par Config Status
         *  Optional.
         */
        std::optional<double> focal_length{std::nullopt};
        /*! \brief Configuration derived unique identifier.
         *
         *  \par Config Status
         *  Automatic.
         *
         *  \par Config Note
         *  This SHOULD NOT be entered as a separate configuration file parameter. It is automatically derived from the
         *  camera table index (ex. `[cam0]` would be index 0).
         */
        int index;
        /*! \brief Human readable identifier.
         *
         *  \par Config Status
         *  Required.
         *
         * Used to identify the input data stream from application data sources (ex. ROS topic names) and improve human
         * readability of the diagnostic outputs and dashboard display.
         */
        std::string sensor_name;
    };

    struct Imu {
        static std::optional<Imu> Parse(toml::table const& table);

        std::string sensor_name;
    };

    // TODO(Jack): We need to replace this with the target info type itself! It is a complete copy.
    struct Target {
        static Target Parse(toml::table const& table);

        TargetType target_type;
        std::array<int, 2> size;
        double unit_dimension{1.0};
        bool asymmetric{false};
    };

    // NOTE(Jack): At a high level there are three kinds of config "requirements"
    //
    //  1) required
    //  2) optional with default value
    //  3) optional with no default value
    //
    // This can apply both to the top level configs below and the individual keys within the configs.
    //
    // Required configs are obvious, those are the things that the program cannot run without, for example the name of
    // the camera sensor or the type of target used. Optional with default value configs are those for which we can set
    // a sensible default value; for example the number of threads used by the optimizer can be parameterized if the
    // user wants, but we can also automatically set a value if they don't care.
    //
    // Optional with no default value are the rarest type and are unique because they actually control the pipeline
    // execution; the only one at time of writing (30.06.2026) is Config::Imu. If the IMU table is in the config file
    // then that is the same as the user telling the pipeline to do a extrinsic calibration. If it is not present in the
    // config file then the user is only asking for a camera intrinsic calibration. Whether or not this is easy for the
    // user to understand, only time will tell :)
    //
    // Look at the config_parse.test.cpp for hands-on examples of what valid configs are and are not.
    Application application;
    std::vector<Camera> cameras;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config