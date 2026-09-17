from dash import Input, Output, html

from dashboard.server import app
from dashboard.tools.construct_layouts import camera_layout, imu_layout, rig_layout
from dashboard.tools.workflow import assets_of_type, camera_label, stage_cameras


@app.callback(
    Output("sensor-content-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
    Input("stage-selector", "value"),
)
def render_sensor_panel(asset_id, metadata, stage_id="single_cam"):
    cameras = stage_cameras(metadata, stage_id)
    camera = next(
        (camera for camera in cameras if camera["id"] == asset_id),
        None,
    )
    if not cameras or (camera is None and stage_id == "single_cam"):
        return html.P(
            "Choose a camera to inspect its calibration.", className="empty-state"
        )
    panels = [
        (
            rig_layout(cameras)
            if stage_id in ("multi_cam", "cam_imu")
            else camera_layout(asset_id, camera_label(camera))
        )
    ]
    if stage_id == "cam_imu":
        panels.extend(
            imu_layout(imu["id"], imu["name"], metadata)
            for imu in assets_of_type(metadata, "imu")
        )
    return html.Div(panels, className="sensor-panels")
