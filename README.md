# Reprojection - The future is calibrated!

This is an application for target-based intrinsic camera calibration and extrinsic camera-imu calibration. It is
automatically compatible with ROS1 and ROS2 and is completely dockerized. 

## Build

Pull the git-lfs manged files - this includes a test database which is required by the dashboard unit tests.

    git lfs pull

There are three applications - `ros1`, `ros2`, and `video-file`, build the one you need.

    # ROS1
    ./building/local/build_image.sh --stage ros1-app

    # ROS2
    ./building/local/build_image.sh --stage ros2-app

    # Video file
    ./building/local/build_image.sh --stage video-file-app

The application automatically outputs a pdf report and toml file containing the calibration, but if you want a more
in-depth look into the calibration process you will also want to build the dashboard.

    ./building/local/build_image.sh --stage dashboard

> [!NOTE]
> The `video-file` application only support camera intrinsic calibration. For camera-imu extrinsic calibration you need
> to use the ROS based applications.

## Run

The application accepts four command line arguments:

1) `ros1`/`ros2`/`video-file` - The data input format
2) `--config` - The path to the calibration configuration
3) `--data` - The path to the calibration dataset
4) `--workspace` - The path to a directory where output files can be written to

> [!TIP]
> If the `--workspace` argument is not provided it will default to the data's directory.

An example command to run the `video-file` application is:

    ./building/local/run_application.sh video-file \
        --config /home/user/data/calibration_config.toml \
        --data /home/user/data/target_capture_1.mp4 \
        --workspace /home/user/data/

This will output the calibration toml file and a report pdf to the workspace directory. To run the interactive dashboard
run the following commend (use your workspace's path!):

    ./building/local/run_dashboard.sh /home/user/data/

Then open the link shown in the terminal.

## Configuration

Please use [calibration_config.toml](code/test_data/calibration_config.toml) as the example to build your configuration
file from. That configuration file is used in all integration and smoke testing which mean it stays up to date. Please
adapt this to your data and save your configuration file with your data.

> [!WARNING]
> If you only intend to intrinsically calibrate a camera remove the `[imu]` table from the calibration file.

> [!IMPORTANT]
> For ROS1 and ROS2 data the `sensor_name` must match the image topic being calibrated exactly.

## Calibration target types

The following target types are supported:

1) `aprilgrid3`
    * [aprilgrid3 6x4](media/aprilgrid3_6x4.png)
    * [aprilgrid3 7x5](media/aprilgrid3_7x5.png)
    * [aprilgrid3 8x6](media/aprilgrid3_8x6.png)
2) `checkerboard`
3) `circle_grid`
    1) asymmetric
    2) symmetric

To generate Checkerboard or Circle Grid targets
the [target generator tool](https://calib.io/pages/camera-calibration-pattern-generator)
provided by [calib.io](https://calib.io/) is a great choice.

> [!WARNING]
> Aprilgrid3 is NOT the same as the ubiquitous Aprilgrid used by Kalibr. Reprojection is not compatible with the Kalibr
> style Aprilgrid.

### Configuring asymmetric circle grid

Please add the following entry to your configuration file:

        [target.circle_grid]
        asymmetric = true

## Camera Models

The following camera models are supported:

1) `double_sphere` - [f, cx, cy, xi, alpha]
2) `pinhole` - [f, cx, cy]
3) `pinhole_radtan4` - [f, cx, cy, k1, k2, p1, p2]
4) `unified_camera_model` - [f cx, cy, xi]

All camera models use a single focal length `f` instead of the standard two focal lengths `fx` and `fy`. Please
see this excellent [article](https://www.tangramvision.com/blog/camera-modeling-focal-length-collinearity)
from [Tangram Vision](https://www.tangramvision.com/) for an explanation.

## Tips, Tricks, and Warnings

### Set the log level

Before running the application execute this command in the terminal:

      export SPDLOG_LEVEL=debug

For a list of all log levels please see the official [spdlog](https://github.com/gabime/spdlog) documentation. 
