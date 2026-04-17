import sqlite3
import sys

# NOTE(Jack): We need to do this because the integration test dataset is the TUM dataset-calib-imu4 dataset which uses
# Kalibr compatible Aprilgird2 boards which we do not support. Therefore we need to trick the process into thinking that
# it has already extracted the features, which we did manually and stored inside dataset-calib-imu4_512_16.db3 long ago.
#
# The purpose of this little script is to manually write a feature extraction cache key so that we can trigger a cache
# hit and just load the features from the database instead of trying to calculate them from the images (remember we do
# not support Aprilgrid2 from Kalibr!).
#
# I warn you that this is naturally brittle because anytime the feature extraction cache key changes for any reason, we
# will need to update it manually at the call site of this script. That is not the end of the world, but it is a little
# hacky and we should keep our eyes peeled for possible optimizations in the future.

db_path, new_cache = sys.argv[1], sys.argv[2]

conn = sqlite3.connect(db_path)
conn.execute(
    """
    INSERT INTO calibration_steps (step_name, sensor_name, cache_key)
    VALUES (?, ?, ?)
    """,
    ('feature_extraction', '/cam0/image_raw', new_cache)
)
conn.commit()
conn.close()
