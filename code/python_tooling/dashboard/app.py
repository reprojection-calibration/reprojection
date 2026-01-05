from dash import Dash, dcc, html, Input, Output, State, callback
import os
import plotly.graph_objects as go
from database.load_calibration_poses import load_calibration_poses

# TODO(Jack): Visualize extracted targets

# TODO(Jack): Do not hardcode this
DB_DIR = '../../test_data/'

app = Dash()
app.layout = html.Div([
    html.H2("Reprojection - the future is calibrated."),

    html.Div(
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
        children=[
            html.Label('Load'),
            dcc.Dropdown(id='database-dropdown', placeholder="Select a database", style={"width": "50%"}),
            html.Button('Refresh Database List', id='refresh-database-list-button', n_clicks=0),
        ]
    ),

    html.Div(
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
        children=[
            html.Label('Select'),
            # TODO(Jack): Autoload the first sensor as the default picked value in the dropdown
            dcc.Dropdown(id='sensor-dropdown', placeholder="Select a camera sensor", style={"width": "50%"}),
        ]
    ),

    dcc.Loading(
        id="loading-animation",
        children=[
            dcc.Graph(id='rotation-graph'),
            dcc.Graph(id='translation-graph'),
            dcc.Store(id='database-store'),
        ]
    ),
])


@callback(
    Output('database-dropdown', 'options'),
    Output('database-dropdown', 'value'),
    Input('refresh-database-list-button', 'n_clicks')
)
def refresh_database_list(_):
    if not os.path.exists(DB_DIR):
        return [], ''

    database_names = sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])
    if len(database_names) == 0:
        return [], ''

    return database_names, database_names[0]


@callback(
    Output('database-store', 'data'),
    Input('database-dropdown', 'value')
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    full_path = DB_DIR + db_file
    if not os.path.isfile(full_path):
        return None

    initial, optimized, external = load_calibration_poses(full_path)

    data = {'initial': initial, 'optimized': optimized, 'external': external}

    # Make pandas data frame json serializable which is required for anything sent to the browser in dash
    data = {key: value.to_dict('records') for key, value in data.items()}

    return data


@callback(
    Output('sensor-dropdown', 'options'),
    Output('sensor-dropdown', 'value'),
    Input('database-store', 'data')
)
def refresh_sensor_list(data):
    if not data:
        return [], None

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({row['sensor_name'] for row in data['initial']}))
    if len(sensor_names) == 0:
        return [], ''

    # Set the default dropdown sensor value to the first sensor
    return sensor_names, sensor_names[0]


@callback(
    Output('rotation-graph', 'figure'),
    Output('translation-graph', 'figure'),
    Input('sensor-dropdown', 'value'),
    State('database-store', 'data'),
)
def update_translation_graph(selected_sensor, data):
    if not selected_sensor or not data:
        return {}, {}

    sorted_sensor_subset = sorted([sensor for sensor in data['initial'] if sensor['sensor_name'] == selected_sensor],
                                  key=lambda x: x['timestamp_ns'])
    times = [n['timestamp_ns'] for n in sorted_sensor_subset]

    # TODO(Jack): Eliminate copy and paste here!
    # TODO(Jack): Use numpy arrays here if possible?
    rotations_x = [d['rx'] for d in sorted_sensor_subset]
    rotations_y = [d['ry'] for d in sorted_sensor_subset]
    rotations_z = [d['rz'] for d in sorted_sensor_subset]

    rot_fig = go.Figure()
    rot_fig.add_scatter(x=times, y=rotations_x, mode='lines+markers', name='X', legendgroup='ExternalPose')
    rot_fig.add_scatter(x=times, y=rotations_y, mode='lines+markers', name='Y', legendgroup='ExternalPose')
    rot_fig.add_scatter(x=times, y=rotations_z, mode='lines+markers', name='Z', legendgroup='ExternalPose')
    rot_fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Axis Angle Rotation',
        legend_title_text='Sources'
    )

    translation_x = [d['x'] for d in sorted_sensor_subset]
    translation_y = [d['y'] for d in sorted_sensor_subset]
    translation_z = [d['z'] for d in sorted_sensor_subset]

    # TODO(Jack): Add legend group
    trans_fig = go.Figure()
    trans_fig.add_scatter(x=times, y=translation_x, mode='lines+markers', name='rx')
    trans_fig.add_scatter(x=times, y=translation_y, mode='lines+markers', name='ry')
    trans_fig.add_scatter(x=times, y=translation_z, mode='lines+markers', name='rz')
    trans_fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Translation(m)'
    )

    return rot_fig, trans_fig


if __name__ == '__main__':
    app.run(debug=True)
