from dash import Dash, dcc, html, Input, Output, State, callback
import os
import plotly.graph_objects as go
from database.load_image_frame_data import load_image_frame_data

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
    Input('refresh-database-list-button', 'n_clicks')
)
def refresh_database_list(_):
    if not os.path.exists(DB_DIR):
        return []

    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])


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

    df = load_image_frame_data(full_path)

    # Need to flatten multiindex to allow for json serialization
    df = df.reset_index(drop=True)
    df.columns = ['_'.join(col) if isinstance(col, tuple) else col for col in df.columns]

    return df.to_dict('records')


@callback(
    Output('sensor-dropdown', 'options'),
    Input('database-store', 'data')
)
def refresh_sensor_list(data):
    if not data:
        return []

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_name = sorted(list({row['frame_id_sensor_name'] for row in data}))

    return sensor_name


@callback(
    Output('rotation-graph', 'figure'),
    Output('translation-graph', 'figure'),
    Input('sensor-dropdown', 'value'),
    State('database-store', 'data'),
)
def update_translation_graph(selected_sensor, data):
    if not selected_sensor or not data:
        return {}, {}

    sorted_subset = sorted([sensor for sensor in data if sensor['frame_id_sensor_name'] == selected_sensor], key=lambda x: x['frame_id_timestamp_ns'])
    times = [n['frame_id_timestamp_ns'] for n in sorted_subset]


    # TODO(Jack): Eliminate copy and paste here!
    # TODO(Jack): Use numpy arrays here if possible?
    rotations_x = [d['external_pose_rx'] for d in sorted_subset]
    rotations_y = [d['external_pose_ry'] for d in sorted_subset]
    rotations_z = [d['external_pose_rz'] for d in sorted_subset]

    rot_fig = go.Figure()
    rot_fig.add_scatter(x=times, y=rotations_x, mode='lines+markers', name='X', legendgroup='ExternalPose')
    rot_fig.add_scatter(x=times, y=rotations_y, mode='lines+markers', name='Y', legendgroup='ExternalPose')
    rot_fig.add_scatter(x=times, y=rotations_z, mode='lines+markers', name='Z', legendgroup='ExternalPose')
    rot_fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Axis Angle Rotation',
        legend_title_text='Sources'
    )

    translation_x = [d['external_pose_x'] for d in sorted_subset]
    translation_y = [d['external_pose_y'] for d in sorted_subset]
    translation_z = [d['external_pose_z'] for d in sorted_subset]

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
