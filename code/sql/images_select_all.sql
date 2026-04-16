SELECT sensor_name,
       timestamp_ns,
       data
FROM images
ORDER BY timestamp_ns ASC;