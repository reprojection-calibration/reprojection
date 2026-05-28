from business_logic.toml_conversions import toml_to_intrinsic_array
from database.types import CameraModel
from pathlib import Path

from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_extracted_targets_table
)

import numpy as np
import plotly.graph_objects as go

from io import BytesIO

import plotly.graph_objects as go

from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Image,
    Table,
    TableStyle,
)
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.pagesizes import letter

styles = getSampleStyleSheet()


def plotly_figure_to_image(fig, width, height):
    image_bytes = fig.to_image(
        format="png",
        width=width,
        height=height,
        scale=2,
    )

    return BytesIO(image_bytes)


def build_figure_with_caption(fig, caption):
    image_buffer = plotly_figure_to_image(
        fig,
        width=600,
        height=400,
    )

    image = Image(
        image_buffer,
        width=250,
        height=170,
    )

    caption_paragraph = Paragraph(
        f"<font size=9>{caption}</font>",
        styles["BodyText"],
    )

    return Table(
        [[image], [caption_paragraph]],
        colWidths=[250],
        style=TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 0),
            ]
        ),
    )


def build_camera_report(camera_info, extracted_targets):
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
            result = build_camera_report(camera_info, extracted_targets)

            left = build_figure_with_caption(
                result,
                "Figure 1: Reprojection error distribution.",
            )

            right = build_figure_with_caption(
                result,
                "Figure 2: Coverage analysis.",
            )

            layout = Table(
                [[left, right]],
                colWidths=[260, 260],
                style=TableStyle(
                    [
                        ("VALIGN", (0, 0), (-1, -1), "TOP"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 0),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                    ]
                ),
            )


            output_name = name.removesuffix(".db3") + ".pdf"
            output_path = workspace_dir / output_name

            doc = SimpleDocTemplate(
                str(output_path),
                pagesize=letter,
            )

            elements = [
                Paragraph("Calibration Report", styles["Heading1"]),
                Spacer(1, 20),
                layout,
            ]

            doc.build(elements)



if __name__ == "__main__":
    main()
