from pathlib import Path

from build_camera_report import run_report_export
from build_camera_toml import run_toml_export


def main():
    # TODO(Jack): We should not hardcode workspace here because than that means it only works in the docker application.
    # We need to find a principled way to pass workspace to both the dashboard and here. It is not hard.
    workspace_dir = Path("/workspace")

    run_toml_export(workspace_dir)
    run_report_export(workspace_dir)


if __name__ == "__main__":
    main()
