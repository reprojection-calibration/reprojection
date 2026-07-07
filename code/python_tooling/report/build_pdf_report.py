import logging
import textwrap
from pathlib import Path

from camera_figures import coverage_figure, error_figure
from imu_figures import imu_delta_time_figure
from pdf_layout import build_two_column_pdf

from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_extracted_targets_table,
    load_reprojection_errors_table,
    load_imu_data_table,
)

log = logging.getLogger("reprojection")


def run_report_export(workspace_dir):
    db_list, _ = refresh_database_list(workspace_dir)

    for entry in db_list:
        db_name = entry["label"]
        db_path = entry["value"]
        log.info(
            "Generating pdf report for:\n%s",
            textwrap.indent(f"Name: {db_name}\nPath: {db_path}", "  "),
        )

        # ERROR(Jack): What do we do if this is None???
        camera_sections = build_camera_sections(db_path)
        imu_sections = build_imu_sections(db_path)

        sections = []
        if camera_sections is not None:
            sections.extend(camera_sections)
        if imu_sections is not None:
            sections.extend(imu_sections)


        output_name = db_name.removesuffix(".db3") + ".pdf"
        output_path = Path(workspace_dir) / output_name

        print(f"\tAssembling pdf and saving to {output_path}")
        build_two_column_pdf(output_path=output_path, camera_sections=sections)


# TODO(Jack): It would really be in our best interest to get a unit test for this function.
def build_camera_sections(db_path):
    camera_info = load_camera_info_table(db_path)
    extracted_targets = load_extracted_targets_table(db_path)
    reprojection_errors = load_reprojection_errors_table(db_path)

    # TODO(Jack): Honestly all we really need to construct the coverage map is the extracted targets. That contains
    # all the information we need. We should refactor this logic here to allow the creation of the most possible
    # figures using limited or partial databases.
    if extracted_targets is None:
        log.info(
            f"Skipping camera pdf report export - missing extracted target data: {db_path}."
        )
        return None

    # If there is no camera info we just want an empty dict as this is technically a valid state for the coverage
    # figure because it will just use the min and max values of the extracted feature to set the bounds.
    camera_info_map = (
        camera_info.set_index("sensor_name").to_dict("index")
        if camera_info is not None
        else {}
    )

    camera_sections = []
    for sensor_name in extracted_targets["sensor_name"].unique():
        print(f"\tProcessing sensor {sensor_name}")

        camera_info_i = camera_info_map.get(sensor_name)
        extracted_targets_i = extracted_targets[
            extracted_targets["sensor_name"] == sensor_name
            ]
        if extracted_targets_i.empty:
            # NOTE(Jack): This is unique here because if there are no targets then we cannot do anything at all so we
            # completely bypass the figure generation for this camera.
            continue

        coverage_figure_i = coverage_figure(camera_info_i, extracted_targets_i)

        # TODO(Jack): This protecting and checking of data validity conditions everywhere is getting verbose and ugly!
        # Should we instead iterate over the same data structure we use in the dashboard?
        if reprojection_errors is None or reprojection_errors.empty:
            error_figure_i = None
        else:
            # We only want to show the final optimized result so we hardcode bundle_adjustment
            reprojection_errors_i = reprojection_errors[
                (reprojection_errors["sensor_name"] == sensor_name)
                & (reprojection_errors["step_name"] == "bundle_adjustment")
                ]

            if reprojection_errors_i.empty or camera_info_i is None:
                error_figure_i = None
            else:
                error_figure_i = error_figure(
                    camera_info_i,
                    extracted_targets_i,
                    reprojection_errors_i,
                )

        camera_section_i = {
            "sensor_name": sensor_name,
            "rows": [
                (
                    {
                        "fig": coverage_figure_i,
                        "caption": "Extracted target pixel coverage.",
                    },
                    {
                        "fig": error_figure_i,
                        "caption": "Reprojection error magnitude.",
                    },
                ),
            ],
        }

        camera_sections.append(camera_section_i)

    return camera_sections

def build_imu_sections(db_path):
    imu_data = load_imu_data_table(db_path)


    if imu_data is None:
        log.info(
            f"Skipping IMU pdf report export - missing IMU data: {db_path}."
        )
        return None

    imu_sections = []
    for sensor_name in imu_data["sensor_name"].unique():
        print(f"\tProcessing sensor {sensor_name}")

        imu_data_i = imu_data[
            imu_data["sensor_name"] == sensor_name
            ]
        if imu_data_i.empty:
            continue

        time_delta_figure_i = imu_delta_time_figure(imu_data_i)

        imu_section_i = {
            "sensor_name": sensor_name,
            "rows": [
                (
                    {
                        "fig": time_delta_figure_i,
                        # TODO(Jack): Naming!
                        "caption": "Measurement time delta.",
                    },
                    {
                        "fig": None,
                        "caption": "",
                    },
                ),
            ],
        }

        imu_sections.append(imu_section_i)

    return imu_sections

