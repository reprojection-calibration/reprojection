def count_structure(d):
    if not isinstance(d, dict):
        return None

    if d and all(isinstance(k, int) for k in d.keys()):
        return len(d)

    return {k: count_structure(v) for k, v in d.items()}
