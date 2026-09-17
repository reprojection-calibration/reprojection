"""Shared camera playback without assuming equal sample times or frame rates."""

from bisect import bisect_left

from dashboard.tools.selection import selected_targets, table_rows


def synchronized_targets(
    workflow_data, camera_ids, step_id, stage_id, max_offset_ms=20
):
    targets = {
        (row["asset_id"], row["step_id"], int(row["timestamp_ns"])): row
        for asset_id in camera_ids
        for row in selected_targets(workflow_data, asset_id, step_id)
    }
    errors = (
        table_rows(workflow_data, "reprojection_errors", step_id=step_id)
        if step_id is not None
        else []
    )
    if stage_id == "multi_cam" and errors:
        frames = {}
        for error in errors:
            key = (
                error["asset_id"],
                error["source_step_id"],
                int(error["sample_timestamp_ns"]),
            )
            if key in targets:
                timestamp = int(error["frame_timestamp_ns"])
                frames.setdefault(timestamp, {})[str(error["asset_id"])] = targets[key]
        return [
            dict(timestamp_ns=str(timestamp), cameras=cameras, time_basis="frame")
            for timestamp, cameras in sorted(frames.items())
        ]

    # Spline results have a pose at each sample time, not a common rig frame.
    # Keep every observed time, then select the nearest sample independently for
    # each camera. A bounded match avoids showing stale detections across gaps.
    if errors:
        result_keys = {
            (row["asset_id"], row["source_step_id"], int(row["sample_timestamp_ns"]))
            for row in errors
        }
        targets = {key: row for key, row in targets.items() if key in result_keys}
    samples = {
        asset_id: sorted(
            (row for row in targets.values() if row["asset_id"] == asset_id),
            key=lambda row: (int(row["timestamp_ns"]), row["step_id"]),
        )
        for asset_id in camera_ids
    }
    timestamps = {
        asset_id: [int(row["timestamp_ns"]) for row in rows]
        for asset_id, rows in samples.items()
    }
    timeline = sorted(
        {timestamp for times in timestamps.values() for timestamp in times}
    )
    tolerance = round(
        max(0, max_offset_ms if max_offset_ms is not None else 20) * 1_000_000
    )
    frames = []
    for timestamp in timeline:
        cameras = {}
        for asset_id, times in timestamps.items():
            index = bisect_left(times, timestamp)
            candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
            if not candidates:
                continue
            # Prefer the earlier sample on an exact tie.
            nearest = min(
                candidates, key=lambda i: (abs(times[i] - timestamp), times[i])
            )
            if abs(times[nearest] - timestamp) <= tolerance:
                cameras[str(asset_id)] = samples[asset_id][nearest]
        frames.append(
            dict(timestamp_ns=str(timestamp), cameras=cameras, time_basis="playback")
        )
    return frames
