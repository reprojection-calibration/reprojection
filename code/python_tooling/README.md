## Testing in CLion

To get a test target in CLion working you need to add a target with the following properties

* Type: "Python tests"
* Target.script_path = /home/stable-genius-gram/github/reprojection-calibration/reprojection/code/python_tooling/tests
* Environmental variables = DB_PATH=/home/stable-genius-gram/github/reprojection-calibration/reprojection/code/test_data/dataset-calib-imu4_512_16.db3
* Python_interpreter.use_specified_interpreter = (.reprojection_venv)