from dataclasses import dataclass

from dash import html

from enum import Enum


class WorkflowType(Enum):
    SingleCam = "single_cam"
    StereoRig = "stereo_rig"
    VisualInertial = "visual_inertial"


@dataclass(frozen=True)
class Stage:
    id: str
    title: str
    description: str
    steps: tuple[str, ...]


STAGES = (
    Stage(
        str(WorkflowType.SingleCam),
        "Individual cameras",
        "Calibrate each camera individually. The targets are extracted, intrinsic initialized, camera poses calculated "
        "using DLT (one frame at a time), and then the entire sequence is optimized using bundle adjustment (poses and "
        "intrinsic).",
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
        str(WorkflowType.StereoRig),
        "Stereo rig",
        "Calibrate all cameras together. The extrinsic from the reference camera (cam0) to all cameras is initialized "
        "using the synchronized poses from the individual camera stage, and then the entire 'rig' is optimized using "
        "bundle adjustment (rig poses and rig extrinsics only)",
        ("stereo_rig_init", "stereo_rig_opt"),
    ),
    Stage(
        str(WorkflowType.VisualInertial),
        "Visual-inertial",
        "Calibrate the reference camera to the IMU. A motion spline is fitted to the reference camera's (cam0) pose, "
        "the extrinsic is initialized using velocity and acceleration constraints, and then the entire sequence is "
        "optimized using a visual-inertial bundle adjustment (cam-imu extrinsic only).",
        (
            "imu_data_loading",
            "spline_init",
            "visual_inertial_init",
            "visual_inertial_opt",
        ),
    ),
)

STEP_LABELS = {
    "pose_init": "Dlt camera pose initialization",
    "bundle_adjustment": "Single camera bundle adjustment",
    "stereo_rig_init": "Stereo rig initialization",
    "stereo_rig_opt": "Stereo rig bundle adjustment",
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
    if stage_id == WorkflowType.SingleCam:
        return bool(cameras)
    if stage_id == WorkflowType.StereoRig:
        return len(cameras) > 1 or bool(stage_step_ids(metadata, stage_id))
    if stage_id == WorkflowType.VisualInertial:
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
    return {
        row["step_id"]
        for row in (metadata or {}).get("counts", [])
        if row["asset_id"] == asset_id
           and row["table"] in ("camera_poses", "reprojection_errors")
           and row["count"] > 0
           and (allowed is None or row["step_id"] in allowed)
    }


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
