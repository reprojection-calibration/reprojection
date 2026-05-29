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


def coverage_figure(camera_info, extracted_targets):
    sensor_name = camera_info["sensor_name"]

    cam_df = extracted_targets[extracted_targets["sensor_name"] == sensor_name]

    all_x = []
    all_y = []
    for row in cam_df.itertuples():
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
            name=sensor_name,
        )
    )

    img_width = camera_info["width"]
    img_height = camera_info["height"]

    fig.update_layout(
        title=f"Pixel Tracks: {sensor_name}",
        template="plotly_white",
        xaxis=dict(
            range=[0, img_width],
            title="x",
        ),
        yaxis=dict(
            range=[img_height, 0],
            title="y",
            scaleanchor="x",
            scaleratio=1,
        ),
    )

    return fig


def main():
    # TODO(Jack): We should not hardcode workspace here because than that means it only works in the docker application.
    # We need to find a principled way to pass workspace to both the dashboard and here. It is not hard.
    workspace_dir = Path("../../code/test_data/")

    db_list, _ = refresh_database_list(workspace_dir)
    for entry in db_list:
        name = entry["label"]
        path = entry["value"]

        camera_infos = load_camera_info_table(path)
        extracted_targets = load_extracted_targets_table(path)

        for i, camera_info in camera_infos.iterrows():
            result1 = coverage_figure(camera_info, extracted_targets)

            output_name = name.removesuffix(".db3") + ".pdf"
            output_path = workspace_dir / output_name

            build_two_column_pdf(
                output_path=Path(output_path),
                rows=[
                    (
                        {
                            "fig": result1,
                            "caption": "Figure 1: Reprojection error distribution.",
                        },
                        {
                            "fig": result1,
                            "caption": "Figure 2: Image coverage.",
                        },
                    ),
                ],
            )


if __name__ == "__main__":
    main()
