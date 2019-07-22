#! /usr/bin/env python

import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    pose = [1.0, 1.25, 1.0, -2.25, -0.3, 1.0, 0.0]

    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(pose))


if __name__ == '__main__':
    main()
