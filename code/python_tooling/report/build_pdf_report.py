import logging
import textwrap
from pathlib import Path

from camera_figures import coverage_figure, error_figure
from dual_use_figures import measurement_delta_time_figures
from pdf_layout import build_two_column_pdf

from dashboard.tools.data_loading import refresh_database_list
from database.data_formatting import (
    parse_workflows,
    process_workflow,
)
from database.sql_table_loading import load_calibration_database

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

        db = load_calibration_database(db_path)
        workflows = parse_workflows(db)

        sections = []
        for workflow in workflows:
            workflow_data = process_workflow(db, workflow)

            camera_section = build_camera_section(workflow, workflow_data)
            imu_section = build_imu_section(workflow, workflow_data)

            if camera_section is not None:
                sections.append(camera_section)
            if imu_section is not None:
                sections.append(imu_section)

        output_name = db_name.removesuffix(".db3") + ".pdf"
        output_path = Path(workspace_dir) / output_name

        log.info(f"Assembling pdf and saving to {output_path}")
        build_two_column_pdf(output_path, sections)


# TODO(Jack): It would really be in our best interest to get a unit test for this function.
def build_camera_section(workflow, workflow_data):
    camera = workflow.assets.get("camera")
    if camera is None:
        return None

    sensor_name = camera["name"]
    log.info(f"Processing sensor {sensor_name}")

    camera_info = workflow_data.get("camera_info")
    extracted_targets = workflow_data.get("extracted_targets")
    reprojection_errors = workflow_data.get("reprojection_errors")
    images = workflow_data.get("images_timestamps")

    # TODO(Jack): Honestly all we really need to construct the coverage map is the extracted targets. That contains
    # all the information we need. We should refactor this logic here to allow the creation of the most possible
    # figures using limited or partial databases.
    if extracted_targets is None or extracted_targets.empty:
        log.info(
            f"Skipping camera pdf report export - missing extracted target data for {sensor_name}."
        )
        return None

    # If there is no camera info we just want an empty dict as this is technically a valid state for the coverage
    # figure because it will just use the min and max values of the extracted feature to set the bounds.
    camera_info_i = (
        camera_info.to_dict()
        if camera_info is not None and not camera_info.empty
        else {}
    )

    coverage_figure_i = coverage_figure(camera_info_i, extracted_targets)

    if reprojection_errors is None or reprojection_errors.empty:
        error_figure_i = None
    else:
        # We only want to show the final optimized result so we hardcode bundle_adjustment.
        bundle_adjustment_step_id = workflow.steps.get("bundle_adjustment")

        if bundle_adjustment_step_id is None:
            reprojection_errors_i = reprojection_errors.iloc[0:0]
        else:
            reprojection_errors_i = reprojection_errors[
                reprojection_errors.index.get_level_values("step_id")
                == bundle_adjustment_step_id
            ]

        if reprojection_errors_i.empty or not camera_info_i:
            error_figure_i = None
        else:
            error_figure_i = error_figure(
                camera_info_i,
                extracted_targets,
                reprojection_errors_i,
            )

    if images is None or images.empty:
        delta_fig_i, histogram_fig_i = (None, None)
    else:
        delta_fig_i, histogram_fig_i = measurement_delta_time_figures(
            images,
            "Camera",
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
            (
                {
                    "fig": delta_fig_i,
                    # TODO(Jack): This caption and the histogram one is copied once here and in the IMU layout. This
                    # will be hard to maintain! We should find a way to only write this in one place.
                    "caption": "Measurement interval timeseries.",
                },
                {
                    "fig": histogram_fig_i,
                    "caption": "Measurement interval histogram.",
                },
            ),
        ],
    }

    return camera_section_i


def build_imu_section(workflow, workflow_data):
    imu = workflow.assets.get("imu")
    if imu is None:
        return None

    sensor_name = imu["name"]
    log.info(f"Processing sensor {sensor_name}")

    imu_data = workflow_data.get("imu_data")

    if imu_data is None or imu_data.empty:
        log.info(
            f"Skipping IMU pdf report export - missing IMU data for {sensor_name}."
        )
        return None

    delta_fig_i, histogram_fig_i = measurement_delta_time_figures(
        imu_data,
        "IMU",
    )

    imu_section_i = {
        "sensor_name": sensor_name,
        "rows": [
            (
                {
                    "fig": delta_fig_i,
                    "caption": "Measurement interval timeseries.",
                },
                {
                    "fig": histogram_fig_i,
                    "caption": "Measurement interval histogram.",
                },
            ),
        ],
    }

    return imu_section_i
