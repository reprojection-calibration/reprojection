import sqlite3, sys

db_path, new_cache = sys.argv[1], sys.argv[2]

conn = sqlite3.connect(db_path)
conn.execute(
    "UPDATE calibration_steps SET cache_key=? WHERE step_name='feature_extraction' AND sensor_name='/cam0/image_raw'",
    (new_cache,)
)
conn.commit()
conn.close()