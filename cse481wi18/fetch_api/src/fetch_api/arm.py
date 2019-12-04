#!/usr/bin/env python

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import math
import moveit_commander
import rospy
import tf

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest 
from tf.listener import TransformListener
from sensor_msgs.msg import JointState

ARM_GROUP_NAME = 'arm'
JOINT_ACTION_SERVER = 'arm_controller/follow_joint_trajectory'
MOVE_GROUP_ACTION_SERVER = 'move_group'
TIME_FROM_START = 5


def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.

    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg

    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._joint_client = actionlib.SimpleActionClient(
            JOINT_ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._joint_client.wait_for_server(rospy.Duration(10))
        self._move_group_client = actionlib.SimpleActionClient(
            MOVE_GROUP_ACTION_SERVER, MoveGroupAction)
        self._move_group_client.wait_for_server(rospy.Duration(10))
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._tf_listener = TransformListener()

    def move_to_joints(self, joint_state):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(ArmJoints.names())
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.extend(joint_state.values())
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._joint_client.send_goal(goal)
        self._joint_client.wait_for_result(rospy.Duration(10))

    def move_to_joint_goal(self,
                           joints,
                           allowed_planning_time=10.0,
                           execution_timeout=rospy.Duration(15.0),
                           group_name='arm',
                           num_planning_attempts=1,
                           plan_only=False,
                           replan=False,
                           replan_attempts=5,
                           tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            joints: A list of (name, value) for the arm joints.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result.
            execution_timeout: float. The maximum duration to wait for an arm
                motion to execute (or for planning to fail completely), in
                seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_joint_goal(*zip(*joints))
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.planner_id = ''
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal = goal_builder.build()
        if goal is not None:
            self._move_group_client.send_goal(goal)
            # success = self._move_group_client.wait_for_result(
                # rospy.Duration(execution_timeout))
            success = self._move_group_client.wait_for_result(execution_timeout)
            print "RESULT *********"
            if not success:
                return moveit_error_string(MoveItErrorCodes.TIMED_OUT)
            result = self._move_group_client.get_result()

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return None
            else:
                return moveit_error_string(result.error_code.val)
        else:
            return moveit_error_string(MoveItErrorCodes.TIMED_OUT)

    def move_to_pose(self,
                     pose_stamped,
                     allowed_planning_time=10.0,
                     execution_timeout=15.0,
                     group_name='arm',
                     num_planning_attempts=1,
                     orientation_constraint=None,
                     plan_only=False,
                     replan=False,
                     replan_attempts=5,
                     tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result.
            execution_timeout: float. The maximum duration to wait for an arm
                motion to execute (or for planning to fail completely), in
                seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            orientation_constraint: moveit_msgs/OrientationConstraint. An
                orientation constraint for the entire path.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        if orientation_constraint is not None:
            goal_builder.orientation_constraint = orientation_constraint
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance

        goal = goal_builder.build()
        if goal is not None:
            self._move_group_client.send_goal(goal)
            self._move_group_client.wait_for_result(
                rospy.Duration(execution_timeout))
            result = self._move_group_client.get_result()

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return None
            else:
                return moveit_error_string(result.error_code.val)
        else:
            return moveit_error_string(MoveItErrorCodes.TIMED_OUT)

    def move_to_pose_with_seed(self,
                     pose_stamped,
                     seed_state=None,
                     trajectory_constraint=[],
                     allowed_planning_time=10.0,
                     execution_timeout=15.0,
                     group_name='arm',
                     num_planning_attempts=1,
                     orientation_constraint=None,
                     plan_only=False,
                     replan=False,
                     replan_attempts=5,
                     tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the gripper.
            seed_state: sensor_msgs/JointState. The start joint state
                to search for solution.
            trajectory_constraint: array of moveit_msgs/Constraints. The trajectory
                constraint.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result.
            execution_timeout: float. The maximum duration to wait for an arm
                motion to execute (or for planning to fail completely), in
                seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            orientation_constraint: moveit_msgs/OrientationConstraint. An
                orientation constraint for the entire path.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        if seed_state:
            goal_builder.start_state.joint_state = seed_state
        if trajectory_constraint:
            goal_builder.add_trajectory_orientation_constraint(trajectory_constraint)

        goal = goal_builder.build()
        if goal is not None:
            self._move_group_client.send_goal(goal)
            self._move_group_client.wait_for_result(
                rospy.Duration(execution_timeout))
            result = self._move_group_client.get_result()

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return None
            else:
                return moveit_error_string(result.error_code.val)
        else:
            return moveit_error_string(MoveItErrorCodes.TIMED_OUT)

    def straight_move_to_pose(self,
                              group,
                              plan):
        """Moves the end-effector to a pose in a straight line.

        Args:
          group: moveit_commander.MoveGroupCommander. The planning group for
            the arm.
          plan: the plan for the arm to follow

        Returns:
            string describing the error if an error occurred, else None.
        """
        # Execute path
        result = group.execute(plan, wait=True)
        if not result:
            return moveit_error_string(MoveItErrorCodes.INVALID_MOTION_PLAN)
        else:
            return None

    def straight_move_to_pose_check(self,
                              group,
                              pose_stamped,
                              ee_step=0.025,
                              jump_threshold=2.0,
                              avoid_collisions=True):
        """Checks if we can move the end-effector to a pose in a straight line.

        Args:
          group: moveit_commander.MoveGroupCommander. The planning group for
            the arm.
          pose_stamped: geometry_msgs/PoseStamped. The goal pose for the
            gripper.
          ee_step: float. The distance in meters to interpolate the path.
          jump_threshold: float. The maximum allowable distance in the arm's
            configuration space allowed between two poses in the path. Used to
            prevent "jumps" in the IK solution.
          avoid_collisions: bool. Whether to check for obstacles or not.

        Returns:
            the generated plan if the arm can move ina straight line, else None.
        """
        # Transform pose into planning frame
        self._tf_listener.waitForTransform(pose_stamped.header.frame_id,
                                           group.get_planning_frame(),
                                           rospy.Time.now(),
                                           rospy.Duration(1.0))
        try:
            pose_transformed = self._tf_listener.transformPose(
                group.get_planning_frame(), pose_stamped)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr('Unable to transform pose from frame {} to {}'.format(
                pose_stamped.header.frame_id, group.get_planning_frame()))
            return moveit_error_string(
                MoveItErrorCodes.FRAME_TRANSFORM_FAILURE)

        # Compute path
        plan, fraction = group.compute_cartesian_path(
            [group.get_current_pose().pose,
             pose_transformed.pose], ee_step, jump_threshold, avoid_collisions)
        if fraction < 1:
            return None
        else:
            return plan

    def check_pose(self,
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: A list of (name, value) for the arm joints if the IK solution
            was found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        joints = []
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                joints.append((name, position))
        return joints

    def compute_fk(self, joint_state):
        """Computes forward kinematics for the given joint state.

        Args:
            joint_state: sensor_msgs/JointState.

        Returns: A geometry_msgs/PoseStamped of the wrist position, False otherwise.
        """
        request = GetPositionFKRequest()
        request.header.frame_id = 'base_link'
        request.fk_link_names = ['wrist_roll_link']
        request.robot_state.joint_state = joint_state
        response = self._compute_fk(request)
        error_str = moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        return response.pose_stamped

    def get_cartesian_path(self,
                           group,
                           start_joint_state,
                           waypoints,
                           ee_step=0.025,
                           jump_threshold=2.0,
                           avoid_collisions=True):
        """Returns a robot trajectory which passes through all the waypoints specified.

        Args:
            group: moveit_commander.MoveGroupCommander. The planning group for
                the arm.
            start_joint_state: sensor_msgs/JointState.
            waypoints: geometry_msgs/Pose[]. Waypoints to pass through.
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the
                gripper.
            ee_step: float. The distance in meters to interpolate the path.
            jump_threshold: float. The maximum allowable distance in the arm's
                configuration space allowed between two poses in the path. Used to
                prevent "jumps" in the IK solution.
            avoid_collisions: bool. Whether to check for obstacles or not.

        Returns: A RobotTrajectory to follow, False otherwise.
        """
        # request = GetCartesianPathRequest()
        # request.header.frame_id = 'base_link'
        # request.start_state.joint_state = start_joint_state
        # request.group_name = ARM_GROUP_NAME
        # request.waypoints = waypoints
        # request.max_step = 0.025
        # request.jump_threshold = 2.0
        # request.avoid_collisions = True
        # response = self._get_cartesian_path(request)
        # error_str = moveit_error_string(response.error_code.val)
        # success = error_str == 'SUCCESS'
        # if not success:
        #     return False
        # return response.solution

        # Compute path
        plan, fraction = group.compute_cartesian_path(
            waypoints, ee_step, jump_threshold, avoid_collisions)
        if fraction < 1:
            return None
        else:
            return plan

    def execute_trajectory(self, group, trajectory):
        """Moves the end-effector along the given trajectory. 

        Args:
            group: moveit_commander.MoveGroupCommander. The planning group for
                the arm.
            trajectory: the RobotTrajectory to follow.

        Return:
            string describing the error if an error occurred, else None.
        """
        result = group.execute(trajectory, wait=True)
        if not result:
            return moveit_error_string(MoveItErrorCodes.INVALID_MOTION_PLAN)
        else:
            return None

    def cancel_all_goals(self):
        self._move_group_client.cancel_all_goals()
        self._joint_client.cancel_all_goals()
