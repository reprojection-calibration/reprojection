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


def main():
    # TODO(Jack): We should not hardcode workspace here because than that means it only works in the docker application.
    # We need to find a principled way to pass workspace to both the dashboard and here. It is not hard.
    workspace_dir = Path("../../code/test_data/")

    db_list, _ = refresh_database_list(workspace_dir)
    for entry in db_list:
        db_path = entry["value"]

        camera_info_df = load_camera_info_table(db_path)
        camera_info_map = camera_info_df.set_index("sensor_name").to_dict("index")
        extracted_target_df = load_extracted_targets_table(db_path)

        camera_sections = []
        for sensor_name in extracted_target_df["sensor_name"].unique():
            camera_info_i = camera_info_map.get(sensor_name)
            extracted_targets_i = extracted_target_df[extracted_target_df["sensor_name"] == sensor_name]

            result1 = coverage_figure(camera_info_i, extracted_targets_i)

            camera_section_i = {
                "sensor_name": sensor_name,
                "rows": [
                    (
                        {
                            "fig": result1,
                            "caption": "Extracted target pixel coverage.",
                        },
                        {
                            "fig": result1,
                            "caption": "Reprojection error distribution.",
                        },
                    ),
                ],
            }

            camera_sections.append(camera_section_i)

        db_name = entry["label"]
        output_name = db_name.removesuffix(".db3") + ".pdf"
        output_path = workspace_dir / output_name

        build_two_column_pdf(output_path=output_path, camera_sections=camera_sections)


if __name__ == "__main__":
    main()
