import sqlite3

from database.sql_statement_loading import load_sql


def execute_sql(db_path, sql_query, params=()):
    with sqlite3.connect(db_path) as conn:
        cursor = conn.execute(sql_query, params)
        row = cursor.fetchone()

    # Because we use "RETURNING id" in some of our sql we need to try to consume the row otherwise the statement will
    # never finish.
    return row[0] if row is not None else None


def construct_test_db(db_path):
    # Enable foreign keys.
    execute_sql(db_path, "PRAGMA foreign_keys = ON;")

    # Metadata/workflow tables.
    execute_sql(db_path, load_sql("assets_table.sql"))
    execute_sql(db_path, load_sql("steps_table.sql"))
    execute_sql(db_path, load_sql("workflow_assets_table.sql"))
    execute_sql(db_path, load_sql("workflow_steps_table.sql"))
    execute_sql(db_path, load_sql("workflows_table.sql"))

    # Make two workflows - one camera intrinsic and one camera-imu extrinsic calibration.
    cam_workflow_id = execute_sql(
        db_path, load_sql("workflows_insert.sql"), ("cam", "cam_signature")
    )
    cam_imu_workflow_id = execute_sql(
        db_path, load_sql("workflows_insert.sql"), ("cam_imu", "cam_imu_signature")
    )

    # Setup metadata/workflows - this does not reflect at all what actual calibration workflows would look like.
    camera_id = execute_sql(db_path, load_sql("assets_insert.sql"), ("camera", 0, ""))
    imu_id = execute_sql(db_path, load_sql("assets_insert.sql"), ("imu", 0, ""))

    # Add the assets to the workflows - note the camera belongs to both workflows.
    execute_sql(
        db_path, load_sql("workflow_assets_insert.sql"), (cam_workflow_id, camera_id)
    )
    execute_sql(
        db_path,
        load_sql("workflow_assets_insert.sql"),
        (cam_imu_workflow_id, camera_id),
    )
    execute_sql(
        db_path, load_sql("workflow_assets_insert.sql"), (cam_imu_workflow_id, imu_id)
    )

    # Add two example steps to the database.
    image_loading_id = execute_sql(
        db_path, load_sql("steps_insert.sql"), ("image_loading", "")
    )
    imu_data_loading_id = execute_sql(
        db_path, load_sql("steps_insert.sql"), ("imu_data_loading", "")
    )

    # Add the steps to the workflows - note the image_loading belongs to both workflows.
    execute_sql(
        db_path,
        load_sql("workflow_steps_upsert.sql"),
        (cam_workflow_id, "image_loading", image_loading_id),
    )
    execute_sql(
        db_path,
        load_sql("workflow_steps_upsert.sql"),
        (cam_imu_workflow_id, "image_loading", image_loading_id),
    )
    execute_sql(
        db_path,
        load_sql("workflow_steps_upsert.sql"),
        (cam_imu_workflow_id, "imu_data_loading", imu_data_loading_id),
    )

    # Calibration artifact tables (only use a subset here to keep things simple).
    execute_sql(db_path, load_sql("imu_data_table.sql"))
    execute_sql(db_path, load_sql("images_table.sql"))

    # Add one piece of data into each table.
    execute_sql(
        db_path, load_sql("images_insert.sql"), (image_loading_id, camera_id, 0, None)
    )
    execute_sql(
        db_path,
        load_sql("imu_data_insert.sql"),
        (imu_data_loading_id, imu_id, 0, 1, 1, 1, 2, 2, 2),
    )
