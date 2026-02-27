SELECT sensor_name,
       timestamp_ns,
       data
FROM extracted_targets
ORDER BY timestamp_ns ASC;