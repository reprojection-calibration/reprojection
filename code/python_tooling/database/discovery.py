import os


def refresh_database_list(db_dir):
    if db_dir is None or not os.path.exists(db_dir):
        return [], ""

    result = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".calib.db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        result.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(result) == 0:
        return [], ""

    return result, result[0]["value"] if result else ""
