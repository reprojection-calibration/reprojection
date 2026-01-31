def extract_timestamps_and_poses_sorted(frames, extract_fn):
    timestamps = []
    values = []
    for timestamp in sorted(frames):
        frame_i = frames[timestamp]

        try:
            value_i = extract_fn(frame_i)
        except KeyError:
            continue

        if value_i is None:
            continue

        timestamps.append(timestamp)
        values.append(value_i)

    return timestamps, values


def timestamps_to_elapsed_seconds(timestamps_ns):
    if len(timestamps_ns) == 0:
        return []

    # WARN(Jack): The input to this function must be sorted from smallest to largest!
    t0 = timestamps_ns[0]
    timestamps_human_readable = [(t - t0) / 1e9 for t in timestamps_ns]

    return timestamps_human_readable


def calculate_ticks_from_timestamps(timestamps_ns, step=5):
    if len(timestamps_ns) == 0:
        return [], [], []

    # WARN(Jack): The input to this function must be sorted from smallest to largest!
    elapsed_time_s = timestamps_to_elapsed_seconds(timestamps_ns)
    max_time = elapsed_time_s[-1]
    target_times = range(0, int(max_time) + 1, step)

    # NOTE(Jack): The idxs are consumed by the slider bar which is indexed in ticker/counter space  and the seconds
    # output is consumed by the pose graphs where we plot data indexed by time.
    tickvals_idx = []
    tickvals_s = []
    ticktext = []
    for target in target_times:
        closest_index = min(
            range(len(elapsed_time_s)), key=lambda i: abs(elapsed_time_s[i] - target)
        )

        # WARN(Jack): Technically we are introducing an approximation here by using the "closest index". For high
        # frequency data I do not think we will ever notice a problem, but we should be aware of this.
        tickvals_idx.append(closest_index)
        tickvals_s.append(elapsed_time_s[closest_index])
        ticktext.append(f"{target}s")

    return tickvals_idx, tickvals_s, ticktext
