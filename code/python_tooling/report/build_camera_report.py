from io import BytesIO
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.platypus import (
    Image,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_extracted_targets_table,
    load_reprojection_errors_table,
)

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
        ),
    )

    return fig


def build_reprojection_error_figure(
    extracted_targets, reprojection_errors, sensor_name, image_width, image_height
):
    targets = extracted_targets[extracted_targets["sensor_name"] == sensor_name]

    # WARN(Jack): Hardcode to only do the reprojection errors of the camera_nonlinear_refinement step
    errors = reprojection_errors[
        (reprojection_errors["sensor_name"] == sensor_name)
        & (reprojection_errors["step_name"] == "camera_nonlinear_refinement")
    ]

    rows = targets.merge(
        errors,
        on=["sensor_name", "timestamp_ns"],
        suffixes=("_target", "_error"),
    )

    image_center = np.array(
        [
            image_width / 2.0,
            image_height / 2.0,
        ]
    )

    projected_points = []
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

        pixel_distances = np.linalg.norm(
            pixel_vectors,
            axis=1,
        )

        valid = pixel_distances > 0.0

        directions = pixel_vectors[valid] / pixel_distances[valid, None]

        magnitudes = np.linalg.norm(
            errors[valid],
            axis=1,
        )

        projected = directions * magnitudes[:, None]

        projected_points.append(projected)

    if not projected_points:
        return go.Figure()

    projected_points = np.concatenate(
        projected_points,
        axis=0,
    )

    plot_x = projected_points[:, 0]
    plot_y = projected_points[:, 1]

    max_abs = np.max(np.abs(projected_points))

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=plot_x,
            y=plot_y,
            mode="markers",
            marker={
                "size": 5,
            },
        )
    )

    fig.update_layout(
        title=f"camera_nonlinear_refinement - {sensor_name}",
        template="plotly_white",
        xaxis={
            "title": "Projected reprojection error [px]",
            "range": [-max_abs, max_abs],
            "scaleratio": 1,
        },
        yaxis={
            "title": "Projected reprojection error [px]",
            "range": [-max_abs, max_abs],
        },
    )

    fig.add_hline(
        y=0,
        line_width=1,
    )

    fig.add_vline(
        x=0,
        line_width=1,
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
        reprojection_errors = load_reprojection_errors_table(path)

        for i, camera_info in camera_infos.iterrows():
            result1 = coverage_figure(camera_info, extracted_targets)

            sensor_name = camera_info["sensor_name"]
            result2 = build_reprojection_error_figure(
                extracted_targets=extracted_targets,
                reprojection_errors=reprojection_errors,
                sensor_name=sensor_name,
                image_width=512,
                image_height=512,
            )

            left = build_figure_with_caption(
                result1,
                "Figure 1: Reprojection error distribution.",
            )

            right = build_figure_with_caption(
                result2,
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
