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
        (cam_workflow_id, image_loading_id, "image_loading", ""),
    )
    execute_sql(
        db_path,
        load_sql("workflow_steps_upsert.sql"),
        (cam_imu_workflow_id, image_loading_id, "image_loading", ""),
    )
    execute_sql(
        db_path,
        load_sql("workflow_steps_upsert.sql"),
        (cam_imu_workflow_id, imu_data_loading_id, "imu_data_loading", ""),
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


def construct_visualization_db(db_path):
    """A current-schema fixture with real source FKs and colliding names/timestamps."""
    from generated.extracted_target_pb2 import ArrayX2dProto, ExtractedTargetProto

    with sqlite3.connect(db_path) as conn:
        conn.execute("PRAGMA foreign_keys = ON")
        for table in (
            "asset_groups",
            "assets",
            "steps",
            "workflows",
            "workflow_assets",
            "workflow_steps",
            "camera_info",
            "target_info",
            "images",
            "imu_data",
            "extracted_targets",
            "camera_poses",
            "intrinsics",
            "reprojection_errors",
            "imu_errors",
            "extrinsics",
        ):
            conn.execute(load_sql(f"{table}_table.sql"))
        for signature in ("1|2|3|4|", "1|", "2|", "3|", "4|", "1|4|"):
            conn.execute("INSERT INTO asset_groups(signature) VALUES (?)", (signature,))
        conn.executemany(
            'INSERT INTO assets(id, type, "index", name) VALUES (?, ?, ?, ?)',
            [
                (1, "camera", 0, "same"),
                (2, "camera", 1, "same"),
                (3, "target", 0, "board"),
                (4, "imu", 0, "imu"),
            ],
        )
        conn.executemany(
            "INSERT INTO workflows VALUES (?, ?, ?)",
            [
                (1, "cam_imu", "1|2|3|4|"),
                (2, "cam", "2|"),
            ],
        )
        conn.executemany(
            "INSERT INTO workflow_assets VALUES (?, ?)",
            [
                (1, 1),
                (1, 2),
                (1, 3),
                (1, 4),
                (2, 2),
            ],
        )
        steps = [
            (10, "image_loading", "1|"),
            (11, "image_loading", "2|"),
            (20, "feature_extraction", "1|"),
            (21, "feature_extraction", "2|"),
            (30, "bundle_adjustment", "1|"),
            (31, "bundle_adjustment", "2|"),
            (40, "camera_info", "1|"),
            (41, "camera_info", "2|"),
            (42, "target_info", "3|"),
            (50, "imu_data_loading", "4|"),
            (60, "visual_inertial_opt", "1|4|"),
        ]
        for step_id, step_type, signature in steps:
            conn.execute(
                "INSERT INTO steps(id, type) VALUES (?, ?)", (step_id, step_type)
            )
            conn.execute(
                "INSERT INTO workflow_steps VALUES (?, ?, ?, ?)",
                (1, step_id, step_type, signature),
            )
        for step_id in (11, 21, 31, 41):
            _, step_type, signature = next(step for step in steps if step[0] == step_id)
            conn.execute(
                "INSERT INTO workflow_steps VALUES (?, ?, ?, ?)",
                (2, step_id, step_type, signature),
            )
        conn.executemany(
            "INSERT INTO camera_info VALUES (?, ?, ?, ?, ?)",
            [
                (40, 1, "pinhole", 480, 640),
                (41, 2, "pinhole", 720, 1080),
            ],
        )
        conn.execute(
            "INSERT INTO target_info VALUES (42, 3, 'checkerboard', 2, 2, 0.1, 0)"
        )
        for asset_id in (1, 2):
            conn.execute(
                "INSERT INTO intrinsics VALUES (?, ?, 'pinhole', ?)",
                (29 + asset_id, asset_id, "f = 100.0\ncx = 320.0\ncy = 240.0"),
            )
            target = ExtractedTargetProto()
            target.bundle.pixel_rows = 1
            target.bundle.pixel_data.extend([100.0 * asset_id, 150.0])
            target.bundle.point_rows = 1
            target.bundle.point_data.extend([0.1, 0.2, 0.0])
            target.indices_rows = 1
            target.indices_data.extend([0, 0])
            error = ArrayX2dProto(rows=1, array_data=[float(asset_id), 0.0])
            for timestamp in (1700000000000000001, 1700000000000000002):
                conn.execute(
                    "INSERT INTO images VALUES (?, ?, ?, NULL)",
                    (9 + asset_id, asset_id, timestamp),
                )
                conn.execute(
                    "INSERT INTO extracted_targets VALUES (?, ?, ?, ?, ?)",
                    (
                        19 + asset_id,
                        9 + asset_id,
                        asset_id,
                        timestamp,
                        target.SerializeToString(),
                    ),
                )
                conn.execute(
                    "INSERT INTO camera_poses VALUES (?, ?, ?, ?, 0, 0, 0, 1, 2, 3)",
                    (29 + asset_id, 19 + asset_id, asset_id, timestamp),
                )
                conn.execute(
                    "INSERT INTO reprojection_errors VALUES (?, ?, ?, ?, ?, ?)",
                    (
                        29 + asset_id,
                        19 + asset_id,
                        asset_id,
                        timestamp,
                        timestamp,
                        error.SerializeToString(),
                    ),
                )
        conn.execute(
            "INSERT INTO imu_data VALUES (50, 4, 1700000000000000001, 1, 2, 3, 4, 5, 6)"
        )
        conn.execute(
            "INSERT INTO imu_errors VALUES (60, 50, 4, 1700000000000000001, -1, 2, -3, 4, -5, 6)"
        )
        conn.execute("INSERT INTO extrinsics VALUES (60, 1, 4, 0, 0, 0, 1, 2, 3)")
        assert not conn.execute("PRAGMA foreign_key_check").fetchall()


def construct_staged_visualization_db(db_path):
    construct_visualization_db(db_path)
    with sqlite3.connect(db_path) as conn:
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("INSERT INTO asset_groups(signature) VALUES ('1|2|')")
        for step_id, step_type in [(70, "stereo_rig_init"), (71, "stereo_rig_opt")]:
            conn.execute(
                "INSERT INTO steps(id, type) VALUES (?, ?)", (step_id, step_type)
            )
            conn.execute(
                "INSERT INTO workflow_steps VALUES (1, ?, ?, '1|2|')",
                (step_id, step_type),
            )
            conn.execute(
                "INSERT INTO camera_poses SELECT ?, source_step_id, asset_id, timestamp_ns, rx, ry, rz, x, y, z FROM camera_poses WHERE step_id=30",
                (step_id,),
            )
            conn.execute(
                "INSERT INTO reprojection_errors SELECT ?, source_step_id, asset_id, sample_timestamp_ns, frame_timestamp_ns, data FROM reprojection_errors WHERE step_id IN (30, 31)",
                (step_id,),
            )
            conn.execute(
                "INSERT INTO extrinsics VALUES (?, 1, 2, 0, 0, 0, 0.1, 0, 0)",
                (step_id,),
            )
        conn.execute(
            "INSERT INTO camera_poses SELECT 60, source_step_id, asset_id, timestamp_ns, rx, ry, rz, x, y, z FROM camera_poses WHERE step_id=30"
        )
        conn.execute(
            "INSERT INTO reprojection_errors SELECT 60, source_step_id, asset_id, sample_timestamp_ns, frame_timestamp_ns, data FROM reprojection_errors WHERE step_id IN (30, 31)"
        )
        conn.execute("INSERT INTO asset_groups(signature) VALUES ('1|2|4|')")
        conn.execute(
            "UPDATE workflow_steps SET asset_group_signature='1|2|4|' WHERE step_id=60"
        )
        assert not conn.execute("PRAGMA foreign_key_check").fetchall()
