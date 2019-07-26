#! /usr/bin/env python

import rospy
import math
from pprint import pprint
import numpy as np
import fetch_api
from std_msgs.msg import String, Header, ColorRGBA, Bool
from image_geometry import PinholeCameraModel
import tf
import tf.transformations as tft
from tf import TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers
from limb_manipulation_msgs.msg import EzgripperAccess
from sake_gripper import SakeEzGripper
import moveit_commander
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import subprocess
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
from database import Database
import actionlib
import rosbag
import os


# maximum times to retry if a transform lookup fails
TRANSFROM_LOOKUP_RETRY = 10
# the threshold to distinguish joint state points
ARM_TRAJ_TRESHOLD = 0.1

# body parts and their corresponding ID# and actions
BODY_PARTS = {0: "upper right leg", 1: "lower right leg",
              2: "upper left leg", 3: "lower left leg",
              4: "upper right arm", 5: "lower right arm",
              6: "upper left arm", 7: "lower left arm"}

ACTIONS = {0: [],
           1: ["right leg adduction", "right leg abduction", "right leg medial rotation", "right leg lateral rotation"],
           2: [],
           3: ["left leg adduction", "left leg abduction", "left leg medial rotation", "left leg lateral rotation"],
           4: [],
           5: ["right arm elbow extension", "right arm elbow flexion"],
           6: [],
           7: ["left arm elbow extension", "left arm elbow flexion"]
          }

ABBR = {"right leg adduction": "RLAD", "right leg abduction": "RLAB",
        "right leg medial rotation": "RLMR", "right leg lateral rotation": "RLLR",
        "left leg adduction" : "LLAD", "left leg abduction": "LLAB",
        "left leg medial rotation": "LLMR", "left leg lateral rotation": "LLLR",
        "right arm elbow extension": "RAEE", "right arm elbow flexion": "RAEF",
        "left arm elbow extension": "LAEE", "left arm elbow flexion": "LAEF"
       }

# leg medial rotation, leg lateral rotation
# "right arm elbow extension": "RAEE", "right arm elbow flexion": "RAEF", 


# shoulder flexion, shoulder abduction, shoulder adduction, shoulder medial rotation, shoulder lateral rotation, 
# forearm pronation, forearm supination, 
# knee flexion, knee extension (seems to be performed when seated?)



def wait_for_time():
  """
    Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass


class ArTagReader(object):
  def __init__(self):
    self.markers = []  # list of markers (update in real time)
    self.saved_markers = []  # list of markers saved (update only if update() is called)

  def callback(self, msg):
    self.markers = msg.markers

  def update(self):
    self.saved_markers = self.markers

  def get_tag(self, tag):
    """ Returns the marker with id# == tag """
    for marker in self.saved_markers:
      if marker.id == int(tag):
        result = PoseStamped()
        result.pose = marker.pose
        result.header = marker.header
        return result
    return None
  
  def get_list(self):
    """ Returns the list of saved markers """
    return self.saved_markers


class PbdServer():
  """ Server for PBD """

  def __init__(self):
    # controls of Fetch
    self._arm = fetch_api.Arm()
    self._torso = fetch_api.Torso()
    self._head = fetch_api.Head()
    self._base = fetch_api.Base()
    self._fetch_gripper = fetch_api.Gripper()
    # status of the arm: relax or freeze
    self._arm_relaxed = False
    # transformation
    self._tf_listener = tf.TransformListener()
    rospy.sleep(0.1)
    # AR tag reader
    self._reader = ArTagReader()
    self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self._reader.callback)
    # database of actions
    self._db = Database()
    self._db.load()
    # publisher for controls of SAKE gripper
    self._sake_gripper_pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    # motion planning scene
    self._planning_scene = PlanningSceneInterface('base_link')
    self._planning_scene.clear()
    # moveit: query controller
    self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    # # moveit: move group commander
    # moveit_commander.roscpp_initialize(sys.argv)
    # moveit_robot = moveit_commander.RobotCommander()
    # self._moveit_group = moveit_commander.MoveGroupCommander('arm')

    # visualization
    self._viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    # initial position of robot arm
    self._arm_initial_poses = [
        ("shoulder_pan_joint", 1.296), ("shoulder_lift_joint", 1.480), ("upperarm_roll_joint", -0.904), ("elbow_flex_joint", 2.251), 
        ("forearm_roll_joint", -2.021), ("wrist_flex_joint", 1.113), ("wrist_roll_joint", -0.864)]
    
    # orientation constraint
    self._gripper_oc = OrientationConstraint()
    self._gripper_oc.header.frame_id = 'base_link'
    self._gripper_oc.link_name = 'wrist_roll_link'
    self._gripper_oc.orientation.z = -0.707
    self._gripper_oc.orientation.w = 0.707
    self._gripper_oc.absolute_x_axis_tolerance = 0.1
    self._gripper_oc.absolute_y_axis_tolerance = 0.1
    self._gripper_oc.absolute_z_axis_tolerance = 3.14
    self._gripper_oc.weight = 1.0
    # moveit args
    self._kwargs = {
      'allowed_planning_time': 10,
      'execution_timeout': 30,
      'group_name': 'arm',
      'num_planning_attempts': 10,
      'orientation_constraint': self._gripper_oc,
      'replan': True,
      'replan_attempts': 5,
      'tolerance': 0.01
    }

    # current pose of gripper (used in performing actions)
    self._current_pose = None
    # bag file directory
    script_path = os.path.abspath(__file__)
    self._bag_file_dir = os.path.split(script_path)[0][:-4] + '/bags'

    rospy.sleep(0.5)


  def setup(self):
    """ Handler for robot set up """
    print("\nSetting up everything, please wait...\n")
    # set robot's initial state
    self._torso.set_height(0.1)
    self._head.pan_tilt(0, math.pi / 3)

    # collision detection


    # move arm to the initial position (with collision detection)
    self._arm.move_to_joint_goal(self._arm_initial_poses, replan=True)    
    # arm_initial_poses = [1.296, 1.480, -0.904, 2.251, -2.021, 1.113, -0.864]
    # self._arm.move_to_joint_goal(fetch_api.ArmJoints.from_list(arm_initial_poses))


  def shutdown(self):
    """ Handler for robot shutdown """
    print("\nShutting down... Bye :-)\n")
    self.freeze_arm()
    self._planning_scene.removeAttachedObject('sake')
    self._planning_scene.clear()
    self._arm.cancel_all_goals()
    # # moveit: move group commander
    # self._moveit_group.stop()
    # moveit_commander.roscpp_shutdown()

  def attach_sake_gripper(self):
    """
      Attaches SAKE gripper to Fetch's gripper, and updates the planning scene.
    """
    self.freeze_arm()
    # attach SAKE gripper to Fetch's gripper
    self._fetch_gripper.open()  # make sure Fetch's gripper is open
    self._fetch_gripper.close()

    # add SAKE gripper to the planning scene
    self._planning_scene.removeAttachedObject('sake')

    # # real sake gripper mesh
    # sake_gripper_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    # pack_path = subprocess.check_output("rospack find ezgripper_driver", shell=True).replace('\n','')
    # sake_gripper_mesh_file = pack_path + "/meshes/ezgripper_gen2/SAKE_Palm_Dual_Gen2.stl"
    # self._planning_scene.attachMesh('sake', sake_gripper_pose, sake_gripper_mesh_file, True)

    # a box
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
    ]
    self._planning_scene.attachBox('sake', 0.08, 0.07, 0.06, 0, 0, 0,
                             frame_attached_to,
                             frames_okay_to_collide_with)
    self._planning_scene.setColor('sake', 1, 0, 1)
    self._planning_scene.sendColors()

    # calibrate SAKE gripper
    self.calibrate_sake_gripper()

  def remove_sake_gripper(self):
    """
      Removes SAKE gripper from Fetch's gripper, and updates the planning scene.
    """
    self.freeze_arm()
    # remove SAKE gripper from Fetch's gripper
    self._fetch_gripper.close()  # make sure Fetch's gripper is close
    self._fetch_gripper.open()
    # remove SAKE gripper from the planning scene
    self._planning_scene.removeAttachedObject('sake')

  def update_list(self):
    """ Updates the list of markers. """
    self._reader.update()
  
  def get_list(self):
    """ Returns a list of AR tags recognized by the robot. """
    return self._reader.get_list()

  def calibrate_sake_gripper(self):
    print("Calibrating SAKE gripper, please wait...")
    self._sake_gripper_pub.publish(EzgripperAccess(type="calibrate"))

  def hard_close_sake_gripper(self):
    print("Hard closing SAKE gripper, please wait...")
    self._sake_gripper_pub.publish(EzgripperAccess(type="h_close"))

  def soft_close_sake_gripper(self):
    print("Soft closing SAKE gripper, please wait...")
    self._sake_gripper_pub.publish(EzgripperAccess(type="s_close"))
  
  def open_sake_gripper(self):
    print("Opening SAKE gripper, please wait...")
    self._sake_gripper_pub.publish(EzgripperAccess(type="open"))

  def reset(self):
    """ Moves arm to its initial position and calibrates gripper """
    self.freeze_arm()
    self._arm.move_to_joint_goal(self._arm_initial_poses, replan=True)    
    self.calibrate_sake_gripper()

  def goto_part_with_id(self, id):
    """ 
      Moves arm above a body part specified by id.
      Returns true if succeed, false otherwise.
    """
    self.freeze_arm()
    raw_pose = self._get_tag_with_id(id)
    if raw_pose is not None:
      # found marker, move towards it
      goal_pose = PoseStamped()
      goal_pose.pose.position.x = raw_pose.pose.pose.position.x - 0.18  # let SAKE gripper align with the marker
      goal_pose.pose.position.y = raw_pose.pose.pose.position.y
      goal_pose.pose.position.z = raw_pose.pose.pose.position.z + 0.05  # 10cm above
      goal_pose.pose.orientation = raw_pose.pose.pose.orientation  # use the orientation of AR tag
      goal_pose.header.frame_id = "base_link"

      return self._move_arm(goal_pose, final_state=False)
    else:
      # marker with id is not found
      return False
    
  def do_action_with_abbr(self, abbr, id):
    """ 
      Moves arm to perform the action specified by abbr.
      Returns true if succeed, false otherwise.
    """
    self.freeze_arm()

    # move arm with respect to the AR tag
    tag_pose = self._get_tag_with_id(id)
    if tag_pose == None:
      return False

    if not rospy.get_param("use_traj"):
      # check the database for the action
      offset = self._db.get(abbr)
      if self._current_pose and offset:
        return self._move_arm_relative(tag_pose.pose.pose, tag_pose.header, offset[0])

    elif rospy.get_param("use_traj"):
      # check bag files for the trajectory
      bag_file_path = os.path.join(self._bag_file_dir, abbr.lower() + '.bag')
      bag = rosbag.Bag(bag_file_path)
      waypoints = []
      prev_msg = []
      # get the trajectory from bag file
      for topic, msg, t in bag.read_messages(topics=['/joint_states']):
        joint_state = list(msg.position[6:13])
        if len(joint_state) != 0 and (len(prev_msg) == 0 or np.abs(np.sum(np.subtract(joint_state, prev_msg))) > ARM_TRAJ_TRESHOLD):
          prev_msg = joint_state
          # use forward kinematics to find the wrist position
          point = self._arm.compute_fk(msg)
          if point:
            # add the result position
            waypoints.append(point[0])
      bag.close()

      if len(waypoints) < 2:  # empty trajectory
        rospy.logerr("Empty trajectory for action: " + abbr)
        return False

      # follow the trajectory
      prev_pose = self._current_pose
      for i in range(len(waypoints) - 1):
        # calculate offset between the previous point on the trajectory and the current point
        r_pos = waypoints[i].pose  # previous point
        r_mat = self._pose_to_transform(r_pos)
        w_mat = self._pose_to_transform(waypoints[i + 1].pose)
        offset = np.dot(np.linalg.inv(r_mat), w_mat)

        total_offset = offset
        # move arm relative to the previous pose, skip the current waypoint if the current action fails
        action_result = self._move_arm_relative(prev_pose.pose, prev_pose.header, total_offset)
        if action_result is not None:
          prev_pose = action_result
      return action_result is not None

    else:
      rospy.logerr("Invalid action")
      return False

  def record_action_with_abbr(self, abbr, id):
    """
      Records the pose named abbr relative to tag, always overwrites the previous entry (if exists).
      Returns true if succeeds, false otherwise.
    """
    # get tag pose
    tag_pose = self._get_tag_with_id(id)
    if tag_pose == None:
      return False
    # get the pose to be recorded: transformation lookup
    (position, quaternion) = self._tf_lookup()
    if (position, quaternion) == (None, None):
      return False
    # get the transformation, record it
    record_pose = Pose()
    record_pose.position.x = position[0]
    record_pose.position.y = position[1]
    record_pose.position.z = position[2]
    record_pose.orientation.x = quaternion[0]
    record_pose.orientation.y = quaternion[1]
    record_pose.orientation.z = quaternion[2]
    record_pose.orientation.w = quaternion[3]
    # get the offset between the tag pose and the pose to be recorded
    t_pos = tag_pose.pose.pose
    t_mat = self._pose_to_transform(t_pos)
    w_mat = self._pose_to_transform(record_pose)
    offset = np.dot(np.linalg.inv(t_mat), w_mat)
    # add the offset to database
    self._db.add(abbr, offset)
    self._db.save()
    return True

  def relax_arm(self):
    """ Relax the robot arm, if the program is running on the real robot """
    if not rospy.get_param("use_sim") and not self._arm_relaxed:
      goal = QueryControllerStatesGoal()
      state = ControllerState()
      state.name = 'arm_controller/follow_joint_trajectory'
      state.state = ControllerState.STOPPED
      goal.updates.append(state)
      self._controller_client.send_goal(goal)
      self._controller_client.wait_for_result()
      self._arm_relaxed = True

  def freeze_arm(self):
    """ Freeze the robot arm, if the program is running on the real robot """
    if not rospy.get_param("use_sim") and self._arm_relaxed:
      goal = QueryControllerStatesGoal()
      state = ControllerState()
      state.name = 'arm_controller/follow_joint_trajectory'
      state.state = ControllerState.RUNNING
      goal.updates.append(state)
      self._controller_client.send_goal(goal)
      self._controller_client.wait_for_result()
      self._arm_relaxed = False

  def _move_arm_relative(self, ref_pose, ref_header, offset):
    """ 
      Calculates the coordinate of the goal by adding the offset to the given reference pose, and moves the arm to the goal.
      Returns the result of the movement
    """
    # current pose is valid, perform action
    t_mat = self._pose_to_transform(ref_pose)
    # compute the new coordinate
    new_trans = np.dot(t_mat, offset)
    pose = self._transform_to_pose(new_trans)
    goal_pose = PoseStamped()
    goal_pose.pose = pose
    goal_pose.header = ref_header
    # hard close SAKE gripper again to ensure the limb is grasped
    self.hard_close_sake_gripper()
    # visualize goal pose
    marker = Marker(
                type=Marker.SPHERE,
                id=0,
                pose=goal_pose.pose,
                scale=Vector3(0.06, 0.06, 0.06),
                header=goal_pose.header,
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    self._viz_pub.publish(marker)
    # move to the goal position while avoiding unreasonable trajectories!
    if self._move_arm(goal_pose, final_state=True):
      return goal_pose
    return None

  def _move_arm(self, goal_pose, final_state=False):
    """ 
      Moves arm to the specified goal_pose. Returns true if succeed, false otherwise.
    """
    error = None
    if not final_state:
      # simply go to the gaol_pose
      error = self._arm.move_to_pose(goal_pose, **self._kwargs)
      # record the current pose because we still have the "do action" step to do
      self._current_pose = goal_pose
    else:
      # go to goal_pose while avoiding unreasonable trajectories!
      # # OPTION 1: 
      # # moveit: move group commander
      # error = self._arm.straight_move_to_pose(self._moveit_group, goal_pose)

      # OPTION 2: ################################################ TODO: change code below ##########################
      error = self._arm.move_to_pose(goal_pose, **self._kwargs)
      
      # reset current pose to none
      self._current_pose = None
    
    if error is not None:
      self._arm.cancel_all_goals()
      rospy.logerr("Fail to move: {}".format(error))
      return False
    # succeed
    return True

  def _transform_to_pose(self, matrix):
    """ Matrix to pose """
    pose = Pose()
    trans_vector = tft.translation_from_matrix(matrix)
    pose.position = Point(trans_vector[0], trans_vector[1], trans_vector[2])
    quartern = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quartern[0], quartern[1], quartern[2], quartern[3])
    return pose

  def _pose_to_transform(self, pose):
    """ Pose to matrix """
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix

  def _get_tag_with_id(self, id):
    """ Returns the AR tag with the given id, returns None if id not found """
    tag_pose = self._reader.get_tag(id)
    if tag_pose == None:
      rospy.logerr("AR tag lookup error: Invalid ID# " + str(id))
      return None
    return tag_pose

  def _tf_lookup(self):
    """ 
      Lookups the transformation between "base_link" and "wrist_roll_link" (retry up to TRANSFROM_LOOKUP_RETRY times),
      and returns the result
    """
    (position, quaternion) = (None, None)
    count = 0
    while True:
      if (position, quaternion) != (None, None):  # lookup succeeds
        return (position, quaternion)
      elif count >= TRANSFROM_LOOKUP_RETRY:  # exceeds maximum retry times
        rospy.logerr("Fail to lookup transfrom information between 'base_link' and 'wrist_roll_link'")
        return (None, None)
      else: # try to lookup transform information
        try:
          (position, quaternion) = self._tf_listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          count += 1
          continue