# Reprojection - The future is calibrated

Reprojection is an application for target-based camera intrinsic calibration meant to
replace [Kalibr](https://github.com/ethz-asl/kalibr).

The project has over 250 unit tests, multiple integration/smoke tests, 100% code coverage of the core library, and a
comprehensive Github Action CI pipeline. Every single piece of this repository has been painstakingly designed and
battle tested to provide the world's best calibration experience.

#### Notes

* The first release (05.2026) only exposes monocular camera intrinsic calibration but camera-imu extrinsic calibration
  and camera-camera stereo calibration is planned and in progress.

## Build

The three applications provide support for data in ROS1 or ROS2 bags and .mp4 video files, build the one you need.

    # ROS1
    ./building/local/build_image.sh -ts=ros1-app

    # ROS2
    ./building/local/build_image.sh -ts=ros2-app

    # Video file
    ./building/local/build_image.sh -ts=video-file-app

## Run

The application requires four command line arguments:

1) `ros1`/`ros2`/`video-file` - The data input format
2) `--data` - The path to the calibration dataset
3) `--config` - The path to the calibration configuration
4) `--workspace` - The path to a directory where output files can be written to

An example command to run the video-file application is:

    ./building/local/run_application.sh video-file \
        --config /home/user/data/config.toml \
        --data /home/user/data/target_capture_1.mp4 \
        --workspace /home/user/data/

## Configuration

For configuration we use the [toml](https://toml.io/en/) configuration file format. An example minimum required
configuration for monocular camera intrinsic calibration is:

    [sensor]
    camera_name = "/camera/image"
    camera_model = "double_sphere"

    [target]
    pattern_size = [8,6]
    type = "aprilgrid3"

Please adapt this to your data abd save your configuration files using the `.toml` extension.

> [!IMPORTANT]
> For ROS1 and ROS2 data the `camera_name` must match the image topic being calibrated exactly.

## Calibration target types

> [!WARNING]
> Aprilgrid3 is NOT the same as the ubiquitous Aprilgrid used by Kalibr. Reprojection is not compatible with the Kalibr
> style Aprilgrid. You can find compatible Aprilgrid3 target files below. 

The following target types are supported:

1) `aprilgrid3`
2) `checkerboard`
3) `circle_grid`
    1) asymmetric
    2) symmetric

To generate Checkerboard or Circle Grid targets
the [target generator tool](https://calib.io/pages/camera-calibration-pattern-generator)
provided by [calib.io](https://calib.io/) is a great choice.

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
see this excellent [article](https://www.tangramvision.com/blog/camera-modeling-focal-length-collinearity) from the
industry calibration leader [Tangram Vision](https://www.tangramvision.com/) for an explanation. 






