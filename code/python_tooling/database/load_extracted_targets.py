from database.sql_statement_loading import load_sql


def add_extracted_targets_df_to_camera_calibration_data(df, data):
    for index, row in df.iterrows():
        # NOTE(Jack): The reason we are being so string here and throwing errors to kill the program so quickly is
        # simple. Our calibration database has foreign key constraints. This means that a piece of data in the extracted
        # target table CANNOT exist (under no circumstance!!!) unless it has a corresponding entry in the image table.
        # Therefore, if at this point we have a sensor or a timestamp that is not already in 'data' it means we have a
        # big problem.
        sensor = row["sensor_name"]
        if sensor not in data:
            raise RuntimeError(
                f"Sensor {sensor} not present in camera calibration data dictionary",
            )

        timestamp = int(row["timestamp_ns"])
        if timestamp not in data[sensor]["frames"]:
            raise RuntimeError(
                f"Timestamp {timestamp} for sensor {sensor} not present in camera calibration data dictionary"
            )

        target = row["data"]
        data[sensor]["frames"][timestamp].update(
            {
                "extracted_target": {
                    "pixels": target["pixels"],
                    "points": target["points"],
                    "indices": target["indices"],
                }
            }
        )

    return data
