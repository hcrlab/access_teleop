#! /usr/bin/env python

import fetch_api
import rospy
import rosbag
import numpy as np

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    pose1 = [1.0, 1.25, 2.0, -2.25, 0, 1.0, 0.0]
    pose2 = [1.0, 1.25, 2.0, -2.25, 1, 1.0, 0.0]

    arm = fetch_api.Arm()
    # arm.move_to_joints(fetch_api.ArmJoints.from_list(pose1))

    # poses = [fetch_api.ArmJoints.from_list(pose1), fetch_api.ArmJoints.from_list(pose2)]
    # for pose in poses:
    #     arm.move_to_joints(pose)

    bag = rosbag.Bag('/home/maru/catkin_ws/src/limb_manipulation/bags/temp.bag')
    poses = []
    prev_msg = []
    # get the trajectory from bag file
    for topic, msg, t in bag.read_messages(topics=['/joint_states']):
        joint_state = list(msg.position[6:13])
        if len(joint_state) != 0 and (len(prev_msg) == 0 or np.abs(np.sum(np.subtract(joint_state, prev_msg))) > ARM_TRAJ_TRESHOLD):
            prev_msg = joint_state
            pose = fetch_api.ArmJoints.from_list(prev_msg)
            poses.append(pose)
    bag.close()
    # follow the trajectory
    if len(poses) == 0:
        # empty trajectory
        rospy.logerr("Empty trajectory for action: " + abbr)

    for pose in poses:
        arm.move_to_joints(pose)

if __name__ == '__main__':
    main()
