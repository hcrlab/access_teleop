#! /usr/bin/env python

import fetch_api
import rospy

import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    # moveit_commander.roscpp_initialize(sys.argv)
    # moveit_robot = moveit_commander.RobotCommander()
    # moveit_group = moveit_commander.MoveGroupCommander('arm')


    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    arm = fetch_api.Arm()

    # DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
    #                [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]]
    #                # [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
    #                # [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
    #                # [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
    #                # [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
    #                # [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]
    # for vals in DISCO_POSES:
    #   arm.move_to_joints(fetch_api.ArmJoints.from_list(vals))

    goal = [
        ('shoulder_pan_joint', 1.5), ('shoulder_lift_joint', -0.6), ('upperarm_roll_joint', 3.0),
        ('elbow_flex_joint', 1.0), ('forearm_roll_joint', 3.0), ('wrist_flex_joint', 1.0),
        ('wrist_roll_joint', 3.0)
    ]
    error = arm.move_to_joint_goal(goal, 
                allowed_planning_time=120.0,
                execution_timeout=rospy.Duration(60.0),
                num_planning_attempts=5,
                replan=True,
                replan_attempts=5)
    if error is not None:
        arm.cancel_all_goals()
        rospy.logerr('Failed: {}'.format(error))
    else:
        rospy.loginfo('Succeeded')

    # print moveit_group.get_current_pose()
    # print moveit_group.get_current_joint_values()

    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'base_link'

    # goal_pose.pose.position.x = 0
    # goal_pose.pose.position.y = -3
    # goal_pose.pose.position.z = 0
    # goal_pose.pose.orientation.x = 0
    # goal_pose.pose.orientation.y = 0
    # goal_pose.pose.orientation.z = 0
    # goal_pose.pose.orientation.w = 0

    # plan = arm.straight_move_to_pose_check(moveit_group, goal_pose)
    # if plan:
    #     error = arm.straight_move_to_pose(moveit_group, plan)
    # else:
    #     error = 'PLANNING_FAILED'

    # if error:
    #     print error

    # moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
