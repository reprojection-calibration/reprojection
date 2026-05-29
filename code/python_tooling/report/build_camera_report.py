from io import BytesIO
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from pdf_layout import build_two_column_pdf

from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_extracted_targets_table,
    load_reprojection_errors_table,
)


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
        title=f"Pixel Tracks",
        template="plotly_white",
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

    # TODO(Jack): Should we use the calibrated image center instead of the pure
    image_center = np.array([camera_info["width"] / 2.0, camera_info["height"] / 2.0])

    angles = []
    magnitudes = []

    for _, row in rows.iterrows():
        pixels = np.asarray(
            row["data_target"]["pixels"],
            dtype=float,
        )

        errors = np.asarray(
            row["data_error"],
            dtype=float,
        )

        if len(pixels) != len(errors):
            print(
                "Warning: skipping frame because number of pixels "
                f"({len(pixels)}) does not match number of errors "
                f"({len(errors)}) for timestamp "
                f"{row['timestamp_ns']}"
            )
            continue

        pixel_vectors = pixels - image_center
        pixel_distances = np.linalg.norm(pixel_vectors, axis=1)

        valid = pixel_distances > 0.0

        valid_pixel_vectors = pixel_vectors[valid]
        valid_errors = errors[valid]

        frame_angles = np.degrees(
            np.arctan2(
                valid_pixel_vectors[:, 1],
                valid_pixel_vectors[:, 0],
            )
        )

        frame_magnitudes = np.linalg.norm(
            valid_errors,
            axis=1,
        )

        angles.extend(frame_angles)
        magnitudes.extend(frame_magnitudes)

    if not magnitudes:
        return go.Figure()

    max_radius = max(magnitudes)

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

    fig.update_layout(
        title=f"camera_nonlinear_refinement",
        template="plotly_white",
        polar={
            "radialaxis": {
                "title": "Reprojection error [px]",
                "range": [0, max_radius],
            },
            "angularaxis": {
                "rotation": 0,
                "direction": "counterclockwise",
            },
        },
        showlegend=False,
    )

    return fig


def main():
    # TODO(Jack): We should not hardcode workspace here because than that means it only works in the docker application.
    # We need to find a principled way to pass workspace to both the dashboard and here. It is not hard.
    workspace_dir = Path("../../code/test_data/")

    db_list, _ = refresh_database_list(workspace_dir)
    for entry in db_list:
        db_name = entry["label"]
        db_path = entry["value"]
        print(f"Generating pdf report for database {db_name}")

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
                            "caption": "",
                        },
                    ),
                ],
            }

            camera_sections.append(camera_section_i)

        output_name = db_name.removesuffix(".db3") + ".pdf"
        output_path = workspace_dir / output_name

        print(f"\tAssembling pdf and saving to {output_path}")
        build_two_column_pdf(output_path=output_path, camera_sections=camera_sections)


if __name__ == "__main__":
    main()
