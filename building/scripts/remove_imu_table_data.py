import sqlite3
import sys

# NOTE(Jack): EXPLAIN WHY WE ARE DOING THIS!

db_path = sys.argv[1]

try:
    conn = sqlite3.connect(db_path)
except Exception as e:
    print(type(e).__name__, str(e), db_path)
    sys.exit(1)

conn.execute(
    """
    DELETE FROM imu_data;
    """
)
conn.commit()
conn.close()

