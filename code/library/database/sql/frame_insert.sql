INSERT INTO frames (timestamp_ns, sensor_name)
VALUES (?, ?)
ON CONFLICT(timestamp_ns, sensor_name) DO NOTHING;