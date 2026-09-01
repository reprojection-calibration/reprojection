import sqlite3
import argparse
import sys
import os

# NOTE(Jack): We need to do this because the integration test dataset is the TUM dataset-calib-imu4 dataset which uses
# Kalibr compatible Aprilgrid boards which we do not support. Therefore we need to trick the process into thinking that
# it has already extracted the features, which we did manually and stored inside dataset-calib-imu4_512_16.calib.db3 long ago.
#
# The purpose of this little script is to manually write a feature extraction cache key so that we can trigger a cache
# hit and just load the features from the database instead of trying to calculate them from the images (remember we do
# not support Aprilgrid from Kalibr!).
#
# I warn you that this is naturally brittle because anytime the feature extraction cache key changes for any reason, we
# will need to update it manually at the call site of this script. That is not the end of the world, but it is a little
# hacky and we should keep our eyes peeled for possible optimizations in the future.

parser = argparse.ArgumentParser()
parser.add_argument('--db-path', type=str, required=True)
parser.add_argument('--step-ids', nargs='+', type=int, required=True)
parser.add_argument('--cache-keys', nargs='+', type=str, required=True)
args = parser.parse_args()

assert len(args.cache_keys) == len(args.step_ids)
data = list(zip(args.cache_keys, args.step_ids))

if not os.path.isfile(args.db_path):
    print(f"Database does not exist: {args.db_path}")
    sys.exit(1)

try:
    conn = sqlite3.connect(args.db_path)
except Exception as e:
    print(type(e).__name__, str(e), args.db_path)
    sys.exit(1)

conn.executemany(
    """
    UPDATE steps
    SET cache_key = ?
    WHERE id = ?
      AND type = 'feature_extraction';
    """,
    data,
)
conn.commit()
conn.close()
