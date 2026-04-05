import sqlite3, sys

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