SELECT timestamp_ns,
       data
FROM images
WHERE sensor_name = ?
ORDER BY timestamp_ns ASC;