# !/usr/bin/env python

import rospy
from pprint import pprint
import fetch_api


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class MoveByDelta(object):
    def __init__(self, arm, gripper):
        self._arm = arm
        self._gripper = gripper


class MoveByAbsolute(object):
    def __init__(self, arm, gripper):
        self._arm = arm
        self._gripper = gripper


def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    move_by_delta = MoveByDelta(arm, gripper)

    move_by_absolute = MoveByAbsolute(arm, gripper)

if __name__ == "__main__":
    main()
