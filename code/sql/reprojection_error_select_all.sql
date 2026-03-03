SELECT step_name,
       sensor_name,
       timestamp_ns,
       data
FROM reprojection_error
ORDER BY timestamp_ns ASC;