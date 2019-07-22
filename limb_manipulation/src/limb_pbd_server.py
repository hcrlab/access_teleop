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
from moveit_commander import MoveGroupCommander
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import subprocess
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
from database import Database
import actionlib


# body parts and their corresponding ID#
BODY_PARTS = {0: "upper right leg", 1: "lower right leg",
              2: "upper left leg", 3: "lower left leg",
              4: "upper right arm", 5: "lower right arm",
              6: "upper left arm", 7: "lower left arm"}

ACTIONS = {1: ["right leg adduction", "right leg abduction"],
           3: ["left leg adduction", "left leg abduction"]}

ABBR = {"right leg adduction": "RLAD", "right leg abduction": "RLAB",
        "left leg adduction" : "LLAD", "left leg abduction": "LLAB"}

# shoulder flexion, shoulder abduction, shoulder adduction, shoulder medial rotation, shoulder lateral rotation, 
# elbow extension, elbow flexion, 
# forearm pronation, forearm supination, 
# knee flexion, knee extension, 
# leg adduction, leg abduction, leg medial rotation, leg lateral rotation


def wait_for_time():
  """Wait for simulated time to begin.
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
    self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    # status of the arm: relax or freeze
    self._arm_relaxed = False
    # AR tag reader
    self._reader = ArTagReader()
    self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self._reader.callback)
    # database of actions
    self._db = Database()
    self._db.load()
    # transformation
    self._tf_listener = tf.TransformListener()
    # publisher for controls of SAKE gripper
    self._sake_gripper_pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    # motion planning
    self._planning_scene = PlanningSceneInterface('base_link')
    self._planning_scene.clear()

    # initial position of robot arm
    self._arm_initial_poses = [
        ("shoulder_pan_joint", 1.296), ("shoulder_lift_joint", 1.480), ("upperarm_roll_joint", -0.904), ("elbow_flex_joint", 2.251), 
        ("forearm_roll_joint", -2.021), ("wrist_flex_joint", 1.113), ("wrist_roll_joint", -0.864)]
    
    # orientation constraint and moveit args
    self._gripper_oc = OrientationConstraint()
    self._gripper_oc.header.frame_id = 'base_link'
    self._gripper_oc.link_name = 'wrist_roll_link'
    self._gripper_oc.orientation.z = -0.707
    self._gripper_oc.orientation.w = 0.707
    self._gripper_oc.absolute_x_axis_tolerance = 0.1
    self._gripper_oc.absolute_y_axis_tolerance = 0.1
    self._gripper_oc.absolute_z_axis_tolerance = 3.14
    self._gripper_oc.weight = 1.0

    self._kwargs = {
      'allowed_planning_time': 10,
      'execution_timeout': 30,
      'group_name': 'arm',
      'num_planning_attempts': 10,
      # 'orientation_constraint': self._gripper_oc,
      'replan': True,
      'replan_attempts': 5,
      'tolerance': 0.001
    }

    # current pose of gripper (used in performing actions)
    self._current_pose = None


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
    # check the database for the action
    offset = self._db.get(abbr)
    if self._current_pose and offset:
      # current pose is valid, perform action
      # OPTION 1: move arm with respect to current pose
      # tag_pose = self._current_pose

      # OPTION 2: move arm with respect to the AR tag
      tag_pose = self._get_tag_with_id(id)
      if tag_pose == None:
        return False
      t_mat = self._pose_to_transform(tag_pose.pose.pose)
      # compute the new coordinate
      new_trans = np.dot(t_mat, offset[0])
      pose = self._transform_to_pose(new_trans)
      goal_pose = PoseStamped()
      goal_pose.pose = pose
      goal_pose.header = tag_pose.header
      # move to the goal position, hard close SAKE gripper again to ensure the limb is grasped
      self.hard_close_sake_gripper()
      return self._move_arm(goal_pose, final_state=True)
    else:
      rospy.logerr("Invalid action")
      return False

  def record_action_with_abbr(self, abbr, id):
    """ Record the pose relative to tag """
    # get tag pose
    tag_pose = self._get_tag_with_id(id)
    if tag_pose == None:
      return False
    # get the pose to be recorded
    (position, quaternion) = self._tf_listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
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
    """ Relax the robot arm """
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
    """ Freeze the robot arm """
    if not rospy.get_param("use_sim") and self._arm_relaxed:
      goal = QueryControllerStatesGoal()
      state = ControllerState()
      state.name = 'arm_controller/follow_joint_trajectory'
      state.state = ControllerState.RUNNING
      goal.updates.append(state)
      self._controller_client.send_goal(goal)
      self._controller_client.wait_for_result()
      self._arm_relaxed = False

  def _move_arm(self, goal_pose, final_state=False):
    """ 
      Moves arm to the specified goal_pose. Returns true if succeed, false otherwise.
    """
    error = self._arm.move_to_pose(goal_pose, **self._kwargs)
    if error is not None:
      self._arm.cancel_all_goals()
      rospy.logerr("Fail to move: {}".format(error))
      return False
    # success, record the current pose if we still have the "do action" step to do
    if final_state:
      self._current_pose = None
    else:
      self._current_pose = goal_pose
    return True

  def _transform_to_pose(self, matrix):
    pose = Pose()
    trans_vector = tft.translation_from_matrix(matrix)
    pose.position = Point(trans_vector[0], trans_vector[1], trans_vector[2])
    quartern = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quartern[0], quartern[1], quartern[2], quartern[3])
    return pose

  def _pose_to_transform(self, pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix

  def _get_tag_with_id(self, id):
    tag_pose = self._reader.get_tag(id)
    if tag_pose == None:
      rospy.logerr("AR tag lookup error: Invalid ID# " + str(id))
      return None
    return tag_pose