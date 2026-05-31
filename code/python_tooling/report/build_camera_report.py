import numpy as np
import plotly.graph_objects as go
from pdf_layout import build_two_column_pdf

from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_extracted_targets_table,
    load_reprojection_errors_table,
)


def run_report_export(workspace_dir):
    db_list, _ = refresh_database_list(workspace_dir)

    for entry in db_list:
        db_name = entry["label"]
        db_path = entry["value"]
        print(f"Generating pdf camera report for database {db_name}")

        camera_info_df = load_camera_info_table(db_path)
        camera_info_map = camera_info_df.set_index("sensor_name").to_dict("index")
        extracted_target_df = load_extracted_targets_table(db_path)
        reprojection_error_df = load_reprojection_errors_table(db_path)

        camera_sections = []
        for sensor_name in extracted_target_df["sensor_name"].unique():
            print(f"\tProcessing sensor {sensor_name}")

            camera_info_i = camera_info_map.get(sensor_name)
            extracted_targets_i = extracted_target_df[
                extracted_target_df["sensor_name"] == sensor_name
            ]
            coverage_figure_i = coverage_figure(camera_info_i, extracted_targets_i)

            reprojection_errors_i = reprojection_error_df[
                reprojection_error_df["sensor_name"] == sensor_name
            ]
            error_figure_i = error_figure(
                camera_info_i,
                extracted_targets_i,
                reprojection_errors_i,
                "camera_nonlinear_refinement",
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

        output_name = db_name.removesuffix(".db3") + ".pdf"
        output_path = workspace_dir / output_name

        print(f"\tAssembling pdf and saving to {output_path}")
        build_two_column_pdf(output_path=output_path, camera_sections=camera_sections)


def coverage_figure(camera_info, extracted_target_df):
    all_x = []
    all_y = []
    for row in extracted_target_df.itertuples():
        pixels = np.asarray(row.data["pixels"])

        all_x.extend(pixels[:, 0])
        all_y.extend(pixels[:, 1])

    fig = go.Figure()
    fig.add_trace(
        go.Scattergl(
            x=all_x,
            y=all_y,
            mode="markers",
            marker=dict(
                size=3,
                opacity=0.5,
            ),
        )
    )

    if camera_info is not None:
        x_range = [0, camera_info["width"]]
        y_range = [camera_info["height"], 0]
    else:
        x_range = [min(all_x), max(all_x)]
        y_range = [max(all_y), min(all_y)]

    fig.update_layout(
        title=f"Pixel coverage",
        xaxis=dict(
            range=x_range,
            title="x",
            constrain="domain",
        ),
        yaxis=dict(
            range=y_range,
            title="y",
            scaleanchor="x",
            scaleratio=1,
            constrain="domain",
        ),
    )

    return fig


def error_figure(camera_info, extracted_target_df, reprojection_error_df, step_name):
    reprojection_error_df = reprojection_error_df[
        reprojection_error_df["step_name"] == step_name
    ]

    if reprojection_error_df.empty:
        print(f"\t\tNo reprojection errors for {step_name}")
        return None

    assert (
        camera_info is not None
    ), "Camera info was `None` even thought we have reprojection errors... Is the database ok?"

    rows = extracted_target_df.merge(
        reprojection_error_df,
        on=["sensor_name", "timestamp_ns"],
        suffixes=("_target", "_error"),
    )

    # TODO(Jack): Should we use the calibrated image center instead of the pure camera info image dimensions?
    image_center = np.array([camera_info["width"] / 2.0, camera_info["height"] / 2.0])

    angles = []
    magnitudes = []
    for _, row in rows.iterrows():
        pixels = np.asarray(row["data_target"]["pixels"], dtype=float)
        errors = np.asarray(row["data_error"], dtype=float)

        assert len(pixels) == len(
            errors
        ), "Mismatched number if pixels and reprojection errors... Is the database ok?"

        # NOTE(Jack): We want it so that when the user looks at an image or the pixel coverage figure the orientations
        # on the reprojection error bullseye plot align. When calculating the angle we need to account for the fact that
        # the y-axis is flipped and that we want the 0-degree mark to be the top of the image. To solve this here we
        # take the negative of the y coordinate and offset by 270 degrees. If this is really the best solution I am not
        # sure, but it gets the job done!
        pixel_vectors = pixels - image_center
        angles_i = (
            np.degrees(np.arctan2(-pixel_vectors[:, 1], pixel_vectors[:, 0]))
            + 270 % 360
        )

        error_magnitude_i = np.linalg.norm(errors, axis=1)

        angles.extend(angles_i)
        magnitudes.extend(error_magnitude_i)

    fig = go.Figure()

    fig.add_trace(
        go.Scatterpolar(
            r=magnitudes,
            theta=angles,
            mode="markers",
            marker={
                "size": 5,
            },
        )
    )

    # TODO(Jack): It would be smart to dynamically set this (?) but it is required that we show the user that there are
    # errors outside of this max radius that do not show up on the graph. Hardcoding this to a small value here might
    # hide outlier points to the user that they would otherwise expect to see.
    max_radius = 10
    fig.update_layout(
        title=f"Reprojection error bullseye",
        polar={
            "radialaxis": {
                "title": "Reprojection error [px]",
                "range": [0, max_radius],
            },
            "angularaxis": {
                "rotation": 90,
                "direction": "counterclockwise",
            },
        },
        showlegend=False,
    )

    return fig
