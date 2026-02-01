SELECT timestamp_ns,
       data
FROM images
WHERE sensor_name = ?
  AND timestamp_ns >= ?
ORDER BY timestamp_ns ASC;