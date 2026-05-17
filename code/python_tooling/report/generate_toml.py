from pathlib import Path

from build_camera_tomls import build_camera_tomls

# TODO(Jack): Using refresh_database_list here means that it is not really specific to the dashboard itself, maybe we
# should move it to another location.
from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_camera_intrinsics_table,
)


def main():
    # TODO(Jack): We should not hardcode workspace here because than that means it only works in the docker application.
    # We need to find a principled way to pass workspace to both the dashboard and here. It is not hard.
    workspace_dir = Path("/workspace")

    db_list, _ = refresh_database_list(workspace_dir)
    for entry in db_list:
        name = entry["label"]
        path = entry["value"]

        camera_info = load_camera_info_table(path)
        camera_intrinsics = load_camera_intrinsics_table(path)

        result = build_camera_tomls(camera_info, camera_intrinsics)

        if len(result) == 0:
            print("\n\tNo camera intrinsics exported for", name, "\n")
            continue

        output_name = name.removesuffix(".db3") + ".toml"
        output_path = workspace_dir / output_name
        with open(output_path, "w") as f:
            f.write(result)

        print("\n\tCamera intrinsics successfully exported for", name, "\n")


if __name__ == "__main__":
    main()
