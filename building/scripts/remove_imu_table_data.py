import sqlite3
import sys

# NOTE(Jack): The checked in test database which we use for integration and smoke testing already has the IMU data
# inside. This is a problem however for the smoke tests which want to test the entire extrinsic calibration workflow
# (only the ROS application as of 03.07.2026). The problem is that during the smoke test the calibration application
# trys to write the freshly loaded IMU data into the database which already has it. This of course cannot work.
# Therefore, what we do here is remove the IMU data from the database. This means that when the ROS smoke tests try to
# write the IMU data into the database there is no conflict and the test can process with no problem :)

db_path = sys.argv[1]

try:
    conn = sqlite3.connect(db_path)
except Exception as e:
    print(type(e).__name__, str(e), db_path)
    sys.exit(1)

conn.execute(
    """
    DELETE
    FROM imu_data;
    """
)
conn.commit()
conn.close()
