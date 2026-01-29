SELECT timestamp_ns, data
FROM extracted_targets
WHERE sensor_name = ?
ORDER BY timestamp_ns ASC;