## Testing in CLion

To get a test target in CLion working you need to add a target with the following properties

* Type: "Python tests"
* Target.script_path = /home/stable-genius-gram/github/reprojection-calibration/reprojection/code/python_tooling/tests
* Environmental variables = DB_PATH=/home/stable-genius-gram/github/reprojection-calibration/reprojection/code/test_data/dataset-calib-imu4_512_16.db3
* Python_interpreter.use_specified_interpreter = (.reprojection_venv)

## "database.sql not found"

We need to symlink the `code/sql` folder to our python tooling package for local development. In the docker CI workflow
it gets copied over, but for local development there is no reason to actually copy the files. If we copied them then we 
would risk that the developer would make changes to one set of files that would not be reflected in the other set. 

This should be easy to create a symlink and I am probably just missing some obvious points, but it gives me trouble every
time. For me the key to get this to work is to use absolute paths. If you use relative paths this will not work! Here is 
an example command:

    ln --symbolic /<absolute_path>/reprojection/code/sql/ /<absolute_path>/reprojection/code/python_tooling/database/

Adjust the absolute path to reflect your system. 