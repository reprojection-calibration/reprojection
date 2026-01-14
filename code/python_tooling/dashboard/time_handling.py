def extract_timestamps_and_poses_sorted(frames, pose_type):
    timestamps = []
    poses = []
    for timestamp in sorted(frames):
        frame_i = frames[timestamp]

        if 'poses' not in frame_i:
            continue

        if pose_type not in frame_i['poses']:
            continue

        timestamps.append(timestamp)
        poses.append(frame_i['poses'][pose_type])

    return timestamps, poses


def timestamps_to_elapsed_seconds(timestamps_ns):
    t0 = timestamps_ns[0]
    timestamps_human_readable = [(t - t0) / 1e9 for t in timestamps_ns]

    return timestamps_human_readable


def calculate_ticks_from_timestamps(timestamps_ns, step=5):
    timestamps_s = timestamps_to_elapsed_seconds(timestamps_ns)

    max_time = timestamps_s[-1]
    target_times = range(0, int(max_time) + 1, step)

    # NOTE(Jack): The idxs are consumed by the slider bar which is indexed in ticker/counter space  and the seconds
    # output is consumed by the pose graphs where we plot data indexed by time.
    tickvals_idx = []
    tickvals_s = []
    ticktext = []
    for target in target_times:
        closest_index = min(
            range(len(timestamps_s)),
            key=lambda i: abs(timestamps_s[i] - target)
        )

        tickvals_idx.append(closest_index)
        tickvals_s.append(timestamps_s[closest_index])
        ticktext.append(f"{target}s")

    return tickvals_idx, tickvals_s, ticktext
