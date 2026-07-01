import argparse

from build_camera_report import run_report_export
from build_camera_toml import run_toml_export


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    args = parser.parse_args()

    run_toml_export(args.workspace)
    run_report_export(args.workspace)


if __name__ == "__main__":
    main()
