from enum import Enum


# TODO(Jack): What happens here if I do not also inherit from str?
class PoseType(str, Enum):
    Initial = 'initial'
    Optimized = 'optimized'
