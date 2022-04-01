from copy import copy
from enum import Enum

class LaneReference(Enum):
    FAR_LEFT = -1 # -1 since this will be the last element in the array
    INTERIOR = 0
    FAR_RIGHT = 1
    SINGLE = 2
def waypoint_equals(alpha, beta):
    pass
def determine_lane(waypoint) -> LaneReference:
    left = waypoint.get_left_lane()
    right = waypoint.get_right_lane()
    if not left and not right:
        return LaneReference.INTERIOR
    elif not left:
        return LaneReference.FAR_LEFT
    else:
        return LaneReference.FAR_RIGHT

def is_lane_allowed(waypoint, constraint: LaneReference = LaneReference.INTERIOR) -> bool:
    return determine_lane(waypoint) == constraint

