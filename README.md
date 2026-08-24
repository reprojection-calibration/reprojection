# The future is calibrated!

This is an application for target-based sensor calibration. It is completely dockerized and automatically compatible
with ROS1, ROS2, and OpenCV data formats.

This application solves the following problems:

1) **Intrinsic monocular camera calibration**
2) **Camera-imu extrinsic calibration**

The following features are planned and in progress:

1) Stereo camera intrinsic-extrinsic calibration
2) Stereo camera-imu extrinsic calibration
3) Camera-imu temporal calibration

Anytime you have a question or comments please feel free to reach out to me
on [GitHub](https://github.com/reprojection-calibration) or per [email](reprojection.calibration@gmail.com).

## Build

Data can be input as a ROS1 or ROS2 bag file, or in any format supported by the
OpenCV [cv::VideoCapture](https://docs.opencv.org/3.4.20/d8/dfe/classcv_1_1VideoCapture.html) API (ex. an .mp4 video
file or folder of images). The three different application types are:

1) `ros1-app`
2) `ros2-app`
3) `video-file-app` (supports cv::VideoCapture)

An example command to build the video file application is:

    ./building/local/build_image.sh --stage video-file-app

The application automatically outputs a PDF report and toml file containing the calibration, but if you want a more
in-depth look into the calibration process you will also want to build the dashboard.

    ./building/local/build_image.sh --stage dashboard

> [!NOTE]
> The `video-file-app` only support camera intrinsic calibration. For camera-imu extrinsic calibration you need
> to use the ROS based applications.

## Run

The application accepts four command line arguments:

1) `ros1`/`ros2`/`video-file` - The data input format which should match the application you built before
2) `--config` - The path to the calibration configuration
3) `--data` - The path to the calibration dataset
4) `--workspace` - The path to a directory where output files can be written to

An example command to run the `video-file` application is:

    ./building/local/run_application.sh video-file \
        --config /home/user/data/calibration_config.toml \
        --data /home/user/data/target_capture_1.mp4 \
        --workspace /home/user/data/

> [!TIP]
> If the `--workspace` argument is not provided it will default to the data's directory.

This will output the calibration toml file and a report PDF to the workspace directory. To run the interactive dashboard
run the following command (use your workspace's path where the database is):

    ./building/local/run_dashboard.sh /home/user/data/

Then open the link shown in the terminal.

## Configuration

Please use [calibration_config.toml](code/test_data/calibration_config.toml) as the example to build your configuration
file from. This file is used in all integration and smoke testing which mean it stays up to date. Please adapt this to
your data and save it near your data.

> [!WARNING]
> If you only intend to intrinsically calibrate a camera remove the `[imu]` table from the calibration file. Unused or
> invalid configuration keys are not permitted.

> [!IMPORTANT]
> For the ROS applications the `sensor_name` must match the topic exactly.

## Calibration target types

The following target types are supported:

1) `aprilgrid3`
    * [aprilgrid3 4x3](media/targets/aprilgrid3_4x3_v2.png)
    * [aprilgrid3 5x4](media/targets/aprilgrid3_5x4_v2.png)
    * [aprilgrid3 6x5](media/targets/aprilgrid3_6x5_v2.png)
2) `checkerboard`
3) `circle_grid` (symmetric or asymmetric)

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
2) `eucm` - [x, cx, cy, alpha, beta] - "extended unified camera model"
3) `pinhole` - [f, cx, cy]
4) `pinhole_radtan4` - [f, cx, cy, k1, k2, p1, p2]
5) `ucm` - [f cx, cy, alpha] - "unified camera model" - see note below on the Usenko Et al. reformulation.

All camera models use a single focal length `f` instead of the standard two focal lengths `fx` and `fy`. Please see this
excellent [article](https://www.tangramvision.com/blog/camera-modeling-focal-length-collinearity)
from [Tangram Vision](https://www.tangramvision.com/) for an explanation.

We follow the intrinsic parameter conventions from this paper "The Double Sphere Camera Model, Usenko Et al. 2018". Note
that for the "unified camera model" we use the Usenko Et al. proposed numerically stable formulation, and not the
original formulation from "Single view point omnidirectional camera calibration from planar grids, Mei Et al. 2007". The
conversion from one to the other is found in Usenko Et al. section 2.2.

## Tips, Tricks, and Warnings

### Set the log level

Before running the application execute this command in the terminal:

      export SPDLOG_LEVEL=debug

For a list of all log levels please see the official [spdlog](https://github.com/gabime/spdlog) documentation.

## Developers

Pull the git-lfs manged files - this includes a test database which has already extracted targets (used Kalibr)
from a TUM-VIO calibration sequence.

    git lfs pull
