# Reprojection - The future is calibrated

Reprojection is an application for target-based camera intrinsic calibration meant to provide the same capabilities
as the excellent but unmaintained [Kalibr](https://github.com/ethz-asl/kalibr) repository.

#### Notes

* The first release (05.2026) only exposes monocular camera intrinsic calibration. Camera-imu extrinsic calibration
  and camera-camera stereo calibration is planned and in progress.

## Benefits compared to Kalibr

1) Support for ROS1, ROS2, and MP4 - Kalibr is primarily designed around a ROS1 bag based workflow, while Reprojection
   supports ROS1, ROS2, and direct MP4 video inputs.
2) Pure CMake build system - Kalibr relies on the ROS1 catkin build system. Reprojection instead uses a standard
   CMake-based build system without ROS-specific build requirements, simplifying integration into other development
   environments.
3) Robust automatic focal length initialization
4) Database calibration storage - Calibration artifacts such as poses, intrinsics, extracted targets, and
   intermediate optimization results are stored in a SQLite database. This provides a persistent and queryable record of
   the calibration process that can be used for visualization, reproducibility, and long-term storage.
5) Cached execution pipeline - The calibration pipeline is organized into hierarchical processing steps with explicit
   inputs and outputs stored in the database. During repeated runs on the same dataset, only stages affected by changed
   inputs are recomputed.
6) Extensive testing - Reprojection was developed using with test driven development and has 100% unit test code
   coverage of the core calibration library.

## Build

Pull the git-lfs manged files - this includes a test database which is required by the dashboard unit tests.

    git lfs pull

The three applications provide support for data in ROS1 or ROS2 bags and .mp4 video files, build the one you need.

    # ROS1
    ./building/local/build_image.sh --stage ros1-app

    # ROS2
    ./building/local/build_image.sh --stage ros2-app

    # Video file
    ./building/local/build_image.sh --stage video-file-app

To visualize the results you will also need to build the dashboard:

    ./building/local/build_image.sh --stage dashboard

## Run

The application requires four command line arguments:

1) `ros1`/`ros2`/`video-file` - The data input format
2) `--data` - The path to the calibration dataset
3) `--config` - The path to the calibration configuration
4) `--workspace` - The path to a directory where output files can be written to

An example command to run the video-file application is:

    ./building/local/run_application.sh video-file \
        --config /home/user/data/calibration_config.toml \
        --data /home/user/data/target_capture_1.mp4 \
        --workspace /home/user/data/

To generate human-readable outputs run the dashboard and pass the workspace directory path, an example command is:

    ./building/local/run_dashboard.sh /home/user/data/

This command will (1) export the calibrations and generate a pdf report from each database and (2) run the interactive
dashboard. You will find the exported calibrations inside the workspace folder, and you can open the link shown in the
terminal to access the interactive dashboard.

### Set the log level

To get a deeper look into the calibration process set the log level to debug. Before running the application execute
this command in the terminal:

      export SPDLOG_LEVEL=debug

## Configuration

Please use [calibration_config.toml](code/test_data/calibration_config.toml) as the example to build your configuration
file
from. That configuration file is used in all integration and smoke testing which mean it stays up to date. Please
adapt this to your data and save your configuration file with your data.

> [!IMPORTANT]
> For ROS1 and ROS2 data the `sensor_name` must match the image topic being calibrated exactly.

## Calibration target types

The following target types are supported:

1) `aprilgrid3`
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

### Aprilgrid3 target files

* [aprilgrid3 6x4](media/aprilgrid3_6x4.png)
* [aprilgrid3 7x5](media/aprilgrid3_7x5.png)
* [aprilgrid3 8x6](media/aprilgrid3_8x6.png)

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

## Tips and Tricks and Warnings

### Use a target with as many rows/columns as possible

Why? Automatic intrinsic focal length initialization depends on some geometric constraint "black magic" like fitting
circles to the rows/cols and then intersecting those circles or fitting lines to the rows/cols and intersecting those
lines. As you can then imagine the quality of the fitted circle or line increases as the number of data points used
increases.

Trying to fit a line or circle to a row/col with just a couple noisy extracted features is not gonna work, these can be
sensitive algorithms and noise matters. Therefore, please use a target with as many rows/cols as possible.

I recommend at a bare minimum 6x6 for checkerboard and circle grid targets (10x10 an asymmetric circle grid) and 4x4 for
an aprilgrid3 target. If it is possible given your camera and ability to display the targets please use more!

### Use a computer screen to display the calibration target

Sometimes you just want to calibrate but do not have a printer to print out the calibration target... that is annoying.
If that is the case I recommend just showing the target on a computer screen.

Note that when displaying the target you should turn off any "auto blur" effect that the image viewer applies. For
example the GNOME Image Viewer, the default on Ubuntu, has "smooth images" settings which applies a blur that is
intended to make the image more visually appealing. For calibration however this is problem because the feature
extractor needs sharp exact corners.
