import argparse
import logging

from build_camera_report import run_report_export
from build_calibration_toml import run_toml_export


def configure_logging() -> None:
    LOG_FORMAT = "%(levelname)s:%(filename)s:%(lineno)d:%(funcName)s(): %(message)s"

    # The root logger (used by everything else) gets WARNING level logging.
    logging.basicConfig(
        level=logging.WARNING,
        format=LOG_FORMAT,
        force=True,
    )

    # Our project gets INFO level loggin by default.
    logging.getLogger("reprojection").setLevel(logging.INFO)


def main():
    configure_logging()
    logging.info("Running calibration report generation!")

    parser = argparse.ArgumentParser(
        "Generate a calibration report PDF and TOML export from one or more calibration databases."
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
