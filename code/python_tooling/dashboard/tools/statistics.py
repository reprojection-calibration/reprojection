from dash import html


def extract_labeled_metadata(data, parent_keys=None):
    if parent_keys is None:
        parent_keys = []

    statistics = []

    for key, value in data.items():
        current_path = parent_keys + [str(key)]

        if isinstance(value, dict):
            statistics.extend(extract_labeled_metadata(value, current_path))
        else:
            statistics.append((current_path, value))

    return statistics


def build_sensor_statistics_html(sensor_name, metadata):
    if sensor_name is None or metadata is None or sensor_name not in metadata:
        return []

    stats = extract_labeled_metadata(metadata[sensor_name])
    stat_cards = []
    for key, value in stats:
        is_ok = value != 0

        stat_cards.append(
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(
                                style={
                                    "width": "10px",
                                    "height": "10px",
                                    "borderRadius": "50%",
                                    "backgroundColor": "green" if is_ok else "red",
                                    "marginRight": "6px",
                                }
                            ),
                            html.Div(
                                key[0],
                                style={
                                    "fontSize": "13px",
                                    "fontWeight": "500",
                                },
                            ),
                        ],
                        style={
                            "display": "flex",
                            "alignItems": "center",
                            "marginBottom": "6px",
                        },
                    ),

                    html.Div(
                        str(value),
                        style={
                            "fontSize": "18px",
                            "fontWeight": "bold",
                        },
                    ),
                    html.Div(
                        key[-1] if len(key)>1 else "",
                        style={
                            "fontSize": "12px",
                            "color": "#666",
                        },
                    ),
                ],
                style={
                    "minWidth": "140px",
                    "padding": "10px",
                    "backgroundColor": "white",
                    "border": "1px solid #ddd",
                    "borderRadius": "6px",
                    "boxShadow": "0px 1px 2px rgba(0,0,0,0.05)",
                },
            )
        )

    return stat_cards
