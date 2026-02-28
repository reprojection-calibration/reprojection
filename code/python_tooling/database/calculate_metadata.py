def traverse_timeseries_data(d, apply):
    if not isinstance(d, dict):
        return None

    if d and all(isinstance(k, int) for k in d.keys()):
        return apply(d)

    return {k: traverse_timeseries_data(v, apply) for k, v in d.items()}


def count_data(d):
    def process(timestamped_dict):
        return len(timestamped_dict)

    return traverse_timeseries_data(d, process)


# NOTE(Jack): We store the reference timestamps as strings because otherwise dash, when it json serializes this data on
# its way to the browse, will itself cast it to a string. During that process, because we have really large ints here we
# loose some precision which makes correspondence impossible. Therefore, we preempt this mess by ourselves storing the
# the timestamps as strings directly.
def reference_timestamps(d):
    def process(timestamped_dict):
        timestamps = list(timestamped_dict.keys())
        timestamps = sorted(timestamps)
        timestamps = [str(t) for t in timestamps]

        return timestamps

    return traverse_timeseries_data(d, process)
