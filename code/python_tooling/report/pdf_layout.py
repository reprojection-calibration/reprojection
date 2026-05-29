from io import BytesIO

from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.utils import ImageReader
from reportlab.platypus import (
    Image,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

PAGE_SIZE = letter
MARGIN = 40
COLUMN_GAP = 20


def make_document(output_path):
    return SimpleDocTemplate(
        str(output_path),
        pagesize=PAGE_SIZE,
        leftMargin=MARGIN,
        rightMargin=MARGIN,
        topMargin=MARGIN,
        bottomMargin=MARGIN,
    )


def get_column_width(doc):
    return (doc.width - COLUMN_GAP) / 2.0


def plotly_to_image_buffer(fig, width=1000, height=700, scale=2):
    return BytesIO(
        fig.to_image(
            format="png",
            width=width,
            height=height,
            scale=scale,
        )
    )


def make_scaled_image(image_buffer, width):
    image_buffer.seek(0)
    original_width, original_height = ImageReader(image_buffer).getSize()

    image_buffer.seek(0)
    return Image(
        image_buffer,
        width=width,
        height=width * original_height / original_width,
    )


def make_figure_cell(fig, caption, column_width, styles):
    if fig is None:
        return Table([[Spacer(column_width, 1)]], colWidths=[column_width])

    image_buffer = plotly_to_image_buffer(fig)
    image = make_scaled_image(image_buffer, column_width)

    return Table(
        [
            [image],
            [Paragraph(f"<font size=8>{caption}</font>", styles["BodyText"])],
        ],
        colWidths=[column_width],
        style=TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 0),
                ("RIGHTPADDING", (0, 0), (-1, -1), 0),
            ]
        ),
    )


def make_two_column_row(left_cell, right_cell, column_width):
    return Table(
        [[left_cell, right_cell]],
        colWidths=[column_width, column_width],
        style=TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 0),
                ("RIGHTPADDING", (0, 0), (-1, -1), COLUMN_GAP),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 14),
            ]
        ),
    )


def build_two_column_pdf(output_path, camera_sections, title="Calibration Report"):
    doc = make_document(output_path)
    styles = getSampleStyleSheet()
    column_width = get_column_width(doc)

    elements = [
        Paragraph(title, styles["Heading1"]),
        Spacer(1, 16),
    ]

    for section in camera_sections:
        elements.append(
            Paragraph(
                f"Camera: {section['sensor_name']}",
                styles["Heading2"],
            )
        )
        elements.append(Spacer(1, 8))

        for left, right in section["rows"]:
            left_cell = make_figure_cell(
                fig=left["fig"],
                caption=left["caption"],
                column_width=column_width,
                styles=styles,
            )

            right_cell = make_figure_cell(
                fig=right["fig"],
                caption=right["caption"],
                column_width=column_width,
                styles=styles,
            )

            elements.append(
                make_two_column_row(
                    left_cell,
                    right_cell,
                    column_width,
                )
            )

        elements.append(Spacer(1, 16))

    doc.build(elements)
