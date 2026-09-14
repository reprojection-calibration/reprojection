from dataclasses import dataclass

from dash import html

@dataclass(frozen=True)
class Stage:
    id: str
    title: str
    description: str
    steps: tuple[str, ...]


STAGES = (
    Stage(
        "single_cam",
        "Individual cameras",
        "Calibrate each camera independently: extract targets, initialize intrinsics and poses, then refine with bundle adjustment.",
        (
            "image_loading",
            "camera_info",
            "feature_extraction",
            "intrinsic_init",
            "pose_init",
            "bundle_adjustment",
        ),
    ),
    Stage(
        "multi_cam",
        "Stereo rig",
        "Combine the individual camera calibrations: initialize relative camera transforms, then optimize the rig.",
        ("stereo_rig_init", "stereo_rig_opt"),
    ),
    Stage(
        "cam_imu",
        "Visual-inertial",
        "Combine the reference camera's individual calibration with IMU measurements: fit a motion spline, initialize alignment, then optimize.",
        (
            "imu_data_loading",
            "spline_init",
            "visual_inertial_init",
            "visual_inertial_opt",
        ),
    ),
)

STEP_LABELS = {
    "pose_init": "Pose initialization",
    "bundle_adjustment": "Bundle adjustment",
    "spline_init": "Spline initialization",
    "stereo_rig_init": "Rig initialization",
    "stereo_rig_opt": "Rig optimization",
    "visual_inertial_init": "Visual-inertial initialization",
    "visual_inertial_opt": "Visual-inertial optimization",
}


def stage_by_id(stage_id):
    return next((stage for stage in STAGES if stage.id == stage_id), None)


def assets_of_type(metadata, asset_type):
    return sorted(
        (
            asset
            for asset in (metadata or {}).get("assets", [])
            if asset["type"] == asset_type
        ),
        key=lambda asset: (asset.get("index", asset["id"]), asset["id"]),
    )


def camera_label(camera):
    return f"Camera {camera.get('index', camera['id'])} · {camera['name']}"


def stage_step_ids(metadata, stage_id):
    stage = stage_by_id(stage_id)
    return {
        step["step_id"]
        for step in (metadata or {}).get("steps", [])
        if stage is not None and step["type"] in stage.steps
    }


def stage_available(metadata, stage_id):
    cameras = assets_of_type(metadata, "camera")
    if stage_id == "single_cam":
        return bool(cameras)
    if stage_id == "multi_cam":
        return len(cameras) > 1 or bool(stage_step_ids(metadata, stage_id))
    if stage_id == "cam_imu":
        return bool(cameras and assets_of_type(metadata, "imu")) or bool(
            stage_step_ids(metadata, stage_id)
        )
    return False


def stage_cameras(metadata, stage_id):
    if not stage_available(metadata, stage_id):
        return []
    cameras = assets_of_type(metadata, "camera")
    # Calibrate() uses cam_stages.front() for visual-inertial calibration.
    return cameras[:1] if stage_id == "cam_imu" else cameras


def result_step_ids(metadata, asset_id, stage_id=None):
    allowed = stage_step_ids(metadata, stage_id) if stage_id is not None else None
    results = {
        row["step_id"]
        for row in (metadata or {}).get("counts", [])
        if row["asset_id"] == asset_id
        and row["table"] in ("camera_poses", "reprojection_errors")
        and row["count"] > 0
        and (allowed is None or row["step_id"] in allowed)
    }
    return results


def stage_navigation(metadata):
    options = []
    for number, stage in enumerate(STAGES, 1):
        available = stage_available(metadata, stage.id)
        result_cameras = sum(
            bool(result_step_ids(metadata, camera["id"], stage.id))
            for camera in stage_cameras(metadata, stage.id)
        )
        if not available:
            status = (
                "No IMU in this workflow"
                if stage.id == "cam_imu"
                else "Requires multiple cameras"
            )
            if stage.id == "single_cam":
                status = "Select a calibration database"
        elif result_cameras:
            status = f"{result_cameras} camera{'s' if result_cameras != 1 else ''} with results"
        else:
            status = "No pose or reprojection results yet"
        options.append(
            {
                "label": html.Div(
                    [
                        html.Span(f"{number:02d}", className="stage-number"),
                        html.Div([html.Strong(stage.title), html.Small(status)]),
                    ],
                    className="stage-option",
                ),
                "value": stage.id,
                "disabled": not available,
            }
        )
    default = next(
        (option["value"] for option in options if not option["disabled"]), None
    )
    return options, default


def build_stage_overview(stage_id, metadata):
    stage = stage_by_id(stage_id)
    if stage is None:
        return html.Div(
            [
                html.H2("Follow the calibration from camera to rig to IMU"),
                html.P("Choose a database and workflow to inspect its results."),
            ],
            className="empty-state",
        )
    cameras = assets_of_type(metadata, "camera")
    content = [
        html.H2(stage.title),
        html.P(stage.description, className="stage-description"),
    ]
    if stage_id == "single_cam":
        cards = []
        for camera in cameras:
            results = result_step_ids(metadata, camera["id"], stage_id)
            types = {
                step["type"] for step in metadata["steps"] if step["step_id"] in results
            }
            status = (
                "Bundle adjustment results"
                if "bundle_adjustment" in types
                else (
                    "Initial results"
                    if results
                    else "No pose or reprojection results yet"
                )
            )
            cards.append(
                html.Div(
                    [
                        html.Strong(camera_label(camera)),
                        html.Span(
                            status, className="status-available" if results else "muted"
                        ),
                    ],
                    className="camera-progress",
                )
            )
        content.append(html.Div(cards, className="camera-progress-list"))
    elif cameras:
        reference = camera_label(cameras[0])
        note = f"Reference: {reference}. Rig poses are shown on this camera; choose another camera to inspect its reprojection errors."
        if stage_id == "cam_imu":
            imu_names = ", ".join(
                asset["name"] for asset in assets_of_type(metadata, "imu")
            )
            note = f"Reference: {reference}. IMU: {imu_names}. This stage uses the individual camera calibration, not the stereo rig result."
        content.append(html.P(note, className="stage-note"))
    return content
