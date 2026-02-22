def extract_timestamps_and_r6_data_sorted(frames, extract_fn):
    # ERROR(Jack): Discovering this caused me a lot of pain! Bottom line is, the json serialization of the dcc.Stores
    # when they get loaded to the browser converts the frame keys from an int type to a string type -_-, this means
    # that when we try to sort and extract the frame timestamps here, they are strings, and sort and compare according
    # to string rules. One example of how this manifested itself was that for non-padded data, before applying this
    # hack fix here, would sort a value like 100 -after- 1000. Of course, you won't notice this if all your sensor data
    # is on proper unix time, but if your sensor starts counting from zero every time you will notice it right away :)
    frames = {int(k): v for k, v in frames.items()}

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


def timestamps_to_elapsed_seconds(timestamps_ns, t0_ns=None):
    if len(timestamps_ns) == 0:
        return []

    if timestamps_ns != sorted(timestamps_ns):
        raise RuntimeError(
            f"timestamps_ns was not sorted!.",
        )

    if t0_ns is None:
        t0_ns = timestamps_ns[0]
    elif t0_ns > timestamps_ns[0]:
        raise RuntimeError(
            f"t0_ns {t0_ns} was greater than timestamps_ns[0] {timestamps_ns[0]}.",
        )

    timestamps_human_readable = [(t - t0_ns) / 1e9 for t in timestamps_ns]

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
