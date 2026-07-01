import argparse

from build_camera_report import run_report_export
from build_camera_toml import run_toml_export


def main():
    print("Running report generation!")

    parser = argparse.ArgumentParser(
        "Generate a calibration report PDF and camera intrinsics TOML from one or more calibration databases."
    )
    parser.add_argument(
        "--workspace",
        required=True,
        help="Directory containing the calibration database(s). The generated PDF and TOML files are written here.",
    )
    args = parser.parse_args()

    run_toml_export(args.workspace)
    run_report_export(args.workspace)


if __name__ == "__main__":
    main()
