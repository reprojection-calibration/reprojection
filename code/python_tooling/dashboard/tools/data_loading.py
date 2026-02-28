import os
from enum import Enum

from dash import html

from database.calculate_metadata import count_data, reference_timestamps
from database.data_formatting import load_data


def refresh_database_list(db_dir):
    if db_dir is None or not os.path.exists(db_dir):
        return [], ""

    result = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        result.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(result) == 0:
        return [], ""

    return result, result[0]["value"] if result else ""


def load_database(db_file):
    raw_data = load_data(db_file)
    metadata = count_data(raw_data)
    timestamps = reference_timestamps(raw_data)

    return raw_data, metadata, timestamps


def refresh_sensor_list(metadata):
    if metadata is None:
        return [], ""

    result = []
    for sensor_name, value in metadata.items():
        sensor_type = value.get("type")
        if isinstance(sensor_name, Enum):
            sensor_type = sensor_type.name

        result.append(
            {
                "label": f"{sensor_name} ({sensor_type})",
                "value": sensor_name,
            }
        )

    return result, result[0]["value"] if result else ""


def build_sensor_statistics_html(sensor_name, metadata):
    if sensor_name is None or metadata is None:
        return []

    if sensor_name not in metadata:
        return []

    sensor_stats = metadata[sensor_name]

    stat_cards = []
    for key, value in sensor_stats.items():

        is_ok = value != 0

        stat_cards.append(
            html.Div(
                [
                    # Status dot
                    html.Div(
                        style={
                            "width": "10px",
                            "height": "10px",
                            "borderRadius": "50%",
                            "backgroundColor": "green" if is_ok else "red",
                            "marginBottom": "5px",
                        }
                    ),
                    # Value
                    html.Div(
                        str(value),
                        style={
                            "fontSize": "18px",
                            "fontWeight": "bold",
                        },
                    ),
                    # Label
                    html.Div(
                        key,
                        style={
                            "fontSize": "12px",
                            "color": "#666",
                        },
                    ),
                ],
                style={
                    "minWidth": "120px",
                    "padding": "10px",
                    "backgroundColor": "white",
                    "border": "1px solid #ddd",
                    "borderRadius": "6px",
                    "boxShadow": "0px 1px 2px rgba(0,0,0,0.05)",
                },
            )
        )

    return stat_cards
