#! /usr/bin/env python

import rospy
import math
from pprint import pprint
import numpy as np
import fetch_api
from std_msgs.msg import String, Header, ColorRGBA, Bool
from std_srvs.srv import Empty
from image_geometry import PinholeCameraModel
import tf
import tf.transformations as tft
from tf import TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from sensor_msgs.msg import PointCloud2, JointState
from ar_track_alvar_msgs.msg import AlvarMarkers
from limb_manipulation_msgs.msg import EzgripperAccess, WebAppRequest, WebAppResponse
import moveit_commander
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import subprocess
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
from database import Database
import actionlib
import rosbag
import os
from colour import Color
import sys
from shared_teleop_functions_and_vars import dpx_to_distance, delta_modified_stamped_pose
import copy


# maximum times to retry if a transform lookup fails
TRANSFROM_LOOKUP_RETRY = 10
# the threshold to distinguish joint state points
ARM_TRAJ_THRESHOLD = 0.1

# Colors for trajectory visualization
START_COLOR = Color("LightGreen")
END_COLOR = Color("DarkGreen")
TRAJ_HIGHLIGHT_SCALE = Vector3(0.05, 0.008, 0.008)
WAYPOINT_HIGHLIGHT_SCALE = Vector3(0.055, 0.009, 0.009)
WAYPOINT_HIGHLIGHT_COLOR = ColorRGBA(0.0, 0.8, 1.0, 0.8)

# body parts and their corresponding ID# and actions
BODY_PARTS = {0: "right wrist", 1: "lower right leg",
              2: "left wrist", 3: "lower left leg"}

ACTIONS = {0: ["right arm elbow extension", "right arm elbow flexion"],
           1: ["right leg adduction", "right leg abduction", "right leg medial rotation", "right leg lateral rotation"],
           2: ["left arm elbow extension", "left arm elbow flexion"],
           3: ["left leg adduction", "left leg abduction", "left leg medial rotation", "left leg lateral rotation"]
          }

ABBR = {"right leg adduction": "RLAD", "right leg abduction": "RLAB",
        "right leg medial rotation": "RLMR", "right leg lateral rotation": "RLLR",
        "left leg adduction" : "LLAD", "left leg abduction": "LLAB",
        "left leg medial rotation": "LLMR", "left leg lateral rotation": "LLLR",
        "right arm elbow extension": "RAEE", "right arm elbow flexion": "RAEF",
        "left arm elbow extension": "LAEE", "left arm elbow flexion": "LAEF"
       }

# leg medial rotation, leg lateral rotation


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
    self._arm_joints = fetch_api.ArmJoints()
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
    # publisher and subscriber for controls of SAKE gripper
    self._sake_gripper_pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    self._sake_gripper_sub = rospy.Subscriber('/ezgripper_access_status', EzgripperAccess, callback=self._set_sake_gripper_action_status)


    # motion planning scene
    self._planning_scene = PlanningSceneInterface('base_link')
    self._planning_scene.clear()
    # moveit: query controller
    self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    # moveit: move group commander
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_robot = moveit_commander.RobotCommander()
    self._moveit_group = moveit_commander.MoveGroupCommander('arm')

    # visualization
    self._viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    self._viz_markers_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)

    # initial position of robot arm
    self._arm_initial_poses = [
        ("shoulder_pan_joint", 1.296), ("shoulder_lift_joint", 1.480), ("upperarm_roll_joint", -0.904), ("elbow_flex_joint", 2.251), 
        ("forearm_roll_joint", -2.021), ("wrist_flex_joint", -1.113), ("wrist_roll_joint", -0.864)]
    
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
      'allowed_planning_time': 30,
      'execution_timeout': 15,
      'group_name': 'arm',
      'num_planning_attempts': 10,
      # 'orientation_constraint': self._gripper_oc,
      'replan': True,
      'replan_attempts': 5,
      'tolerance': 0.01
    }

    # current pose of gripper (used in performing actions)
    self._current_pose = None
    # bag file directory
    script_path = os.path.abspath(__file__)
    self._bag_file_dir = os.path.split(script_path)[0][:-4] + '/bags'

    # subscriber and publisher for frontend
    self._web_app_request_sub = rospy.Subscriber("web_app_request", WebAppRequest, callback=self.web_app_request_callback)
    self._web_app_response_pub = rospy.Publisher('web_app_response', WebAppResponse, queue_size=5)
    # variables representing the program state
    self._sake_gripper_attached = False
    self._sake_gripper_action_finished = False
    self._sake_gripper_effort = "100"
    self._robot_stopped = False
    self._grasp_position_ready = False
    self._do_position_ready = False
    self._do_position_id = -1
    self._preview_action_abbr = ""
    self._preview_traj = []  # the trajectory being previewed currently

    rospy.sleep(0.5)

  def setup(self):
    """ Handler for robot set up """
    print("\nSetting up everything, please wait...\n")
    # set robot's initial state
    self._torso.set_height(0)
    self._head.pan_tilt(0, 0.8)

    # move arm to the initial position (with collision detection)
    self._arm.move_to_joint_goal(self._arm_initial_poses, replan=True)
    print("\nThe program is ready to use :-)\n")

  def shutdown(self):
    """ Handler for robot shutdown """
    print("\nShutting down... Bye :-)\n")
    # display
    self._viz_markers_pub.publish(MarkerArray(markers=[]))
    # moveit
    self.freeze_arm()
    self._planning_scene.removeAttachedObject('sake')
    self._planning_scene.clear()
    self._arm.cancel_all_goals()
    # database
    self._db.save()
    # moveit: move group commander
    self._moveit_group.stop()
    moveit_commander.roscpp_shutdown()

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
    self.do_sake_gripper_action("calibrate")

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

  def update_env(self, update_octo=True):
    """
      Updates the list of markers, and scan the surroundings to build an octomap.
      Returns false if the update fails, true otherwise.
    """
    # update markers
    self._reader.update()

    if update_octo:
      # update octomap
      # clear previous octomap
      rospy.wait_for_service('clear_octomap')
      try:
        clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
        clear_octo()
      except rospy.ServiceException, e:
        rospy.logerr('Fail to move to the starting position for action: {}'.format(e))
        return False
      # scan the range: pan -0.75~0.75, tilt 0~0.7
      for i in range(6):
        pan = -0.75 + 0.25 * i
        self._head.pan_tilt(pan, 0)
        rospy.sleep(2)
        self._head.pan_tilt(pan, 0.7)
        rospy.sleep(2)
        self._head.pan_tilt(pan, 0)
      # move the head back to initial position
      self._head.pan_tilt(0, 0.7)
    return True
  
  def get_list(self):
    """ Returns a list of AR tags recognized by the robot. """
    return self._reader.get_list()

  def get_db_list(self):
    """ Returns list of entries in the database. """
    return self._db.list()

  def get_db_entry(self, entry):
    """ Returns values associated with the given entry in the database. """
    return self._db.get(entry)

  def delete_db_entry(self, name):
    """ Delete the database entry with the given name """
    self._db.delete(name)
    self._db.save()

  def do_sake_gripper_action(self, command):
    if command == "calibrate":
      self._publish_server_response(msg="Calibrating SAKE gripper, please wait...")
    elif command == "h_close":
      self._publish_server_response(msg="Hard closing SAKE gripper, please wait...")
    elif command == "s_close":
      self._publish_server_response(msg="Soft closing SAKE gripper, please wait...")
    elif command == "open":
      self._publish_server_response(msg="Opening SAKE gripper, please wait...")
    else:
      args = command.split(" ")  # [percentage open, effort]
      if len(args) == 2:
        self._publish_server_response(msg=args[0] + "\% closing SAKE gripper with effort " + args[1] + "\%...")

    # publish command
    self._sake_gripper_pub.publish(EzgripperAccess(type=command))
    # wait for the action to finish if in real
    if not rospy.get_param("use_sim"):
      while not self._sake_gripper_action_finished:
        continue
      # finished, reset
      self._sake_gripper_action_finished = False

  def reset(self):
    """ Moves arm to its initial position and calibrates gripper """
    self.freeze_arm()
    self._arm.move_to_joint_goal(self._arm_initial_poses, replan=True)    
    self.do_sake_gripper_action("calibrate")

  def preview_body_part_with_id(self, id_num):
    """
      Publishes visualization markers to mark the body part with given id.
    """
    raw_pose = self._get_tag_with_id(id_num)
    if raw_pose is not None:
      # visualize goal pose
      marker = Marker(
                  type=Marker.CUBE,
                  id=0,
                  pose=raw_pose.pose.pose,
                  scale=Vector3(0.06, 0.06, 0.06),
                  header=raw_pose.header,
                  color=ColorRGBA(1.0, 0.75, 0.3, 0.8))
      self._viz_pub.publish(marker)

  def goto_part_with_id(self, id_num):
    """ 
      Moves arm above a body part specified by id_num.
      Returns true if succeed, false otherwise.
    """
    self.freeze_arm()
    raw_pose = self._get_tag_with_id(id_num)
    if raw_pose is not None:
      # found marker, move towards it
      self.do_sake_gripper_action("40 " + self._sake_gripper_effort)
      return self._move_arm(self._get_goto_pose(raw_pose), final_state=False)
    else:
      # marker with id_num is not found
      return False

  def preview_action_with_abbr(self, abbr, id_num):
    """
      Publishes visualization markers to preview waypoints on the trajectory with given abbr.
      Saves waypoints extracted from bag file to database if the entry "abbr" doesn't exist.
      Returns the colors of waypoints with respect to the ar tag. Records the positions of
      waypoints with respect to the ar tag.
    """
    # clear previous markers
    self._viz_markers_pub.publish(MarkerArray(markers=[]))
    # check the database for the action
    waypoints = self._db.get(abbr)
    if waypoints == None or len(waypoints) == 0:
      waypoints = self._save_traj_to_db(abbr, id_num)

    self._preview_action_abbr = ""
    self._preview_traj = []
    waypoints_with_respect_to_tag = []
    if waypoints:
      raw_pose = self._get_tag_with_id(id_num)
      if raw_pose is not None:
        prev_pose = self._get_goto_pose(raw_pose)
        # markers
        marker_arr = []
        # marker color gradient
        colors = list(START_COLOR.range_to(END_COLOR, len(waypoints)))
        # visualize the trajectory
        for i in range(len(waypoints)):
          # visualize the current waypoint
          marker = Marker(
                      type=Marker.ARROW,
                      id=i,
                      pose=prev_pose.pose,
                      scale=TRAJ_HIGHLIGHT_SCALE,
                      header=prev_pose.header,
                      color=ColorRGBA(colors[i].red, colors[i].green, colors[i].blue, 0.8))
          marker_arr.append(marker)
          # record the waypoint
          waypoints_with_respect_to_tag.append(str(colors[i].hex))
          self._preview_traj.append(prev_pose)

          if i < len(waypoints) - 1:
            # calculate offset between the current point on the trajectory and the next point
            r_pos = waypoints[i].pose  # current point
            r_mat = self._pose_to_transform(r_pos)
            w_mat = self._pose_to_transform(waypoints[i + 1].pose)
            offset = np.dot(np.linalg.inv(r_mat), w_mat)
            prev_pose = self._move_arm_relative(prev_pose.pose, prev_pose.header, offset=offset, preview_only=True)

        # publish new markers
        self._viz_markers_pub.publish(MarkerArray(markers=marker_arr))
        # record the action name
        self._preview_action_abbr = abbr

    return waypoints_with_respect_to_tag

  def highlight_waypoint(self, highlight_pose, color):
    """ Publishes a marker at the specified location. """
    marker = Marker(
                  type=Marker.ARROW,
                  id=0,
                  pose=highlight_pose.pose,
                  scale=WAYPOINT_HIGHLIGHT_SCALE,
                  header=highlight_pose.header,
                  color=color)
    self._viz_pub.publish(marker)

  def edit_waypoint(self, waypoint_id, delta_x, delta_y, camera):
    """ Temporarily saves the changes to the specified waypoint, and highlights the resulting pose. """
    # calculate the resulting pose
    new_pose = self._compute_pose_by_delta(self._preview_traj[waypoint_id], delta_x, delta_y, camera)
    # save the new pose
    self._preview_traj[waypoint_id] = new_pose

    # preview the new trajectory
    marker_arr = []
    # marker color gradient
    colors = list(START_COLOR.range_to(END_COLOR, len(self._preview_traj)))
    # visualize the trajectory
    for i in range(len(self._preview_traj)):
      # highlight the waypoint that is being editing
      color = WAYPOINT_HIGHLIGHT_COLOR if i == waypoint_id else ColorRGBA(colors[i].red, colors[i].green, colors[i].blue, 0.8)
      marker = Marker(
                    type=Marker.ARROW,
                    id=i,
                    pose=self._preview_traj[i].pose,
                    scale=TRAJ_HIGHLIGHT_SCALE,
                    header=self._preview_traj[i].header,
                    color=color)
      marker_arr.append(marker)
    # clear previous markers
    self._viz_markers_pub.publish(MarkerArray(markers=[]))
    # publish new markers
    self._viz_markers_pub.publish(MarkerArray(markers=marker_arr))

  def modify_traj_in_db(self, cancel_change=True):
    """ Overwrites the previous trajectory in database. """
    if not cancel_change:
      self._db.delete(self._preview_action_abbr)
      self._db.add(self._preview_action_abbr, self._preview_traj)
    self._preview_action_abbr = ""
    self._preview_traj = []

  def do_action_with_abbr(self, abbr, id_num):
    """ 
      Moves arm to perform the action specified by abbr, save the trajectory to database if neccessary.
      Returns true if succeed, false otherwise.
    """
    # preview action
    self.preview_action_with_abbr(abbr, id_num)

    self.freeze_arm()

    # get AR tag information
    tag_pose = self._get_tag_with_id(id_num)
    if tag_pose == None or self._current_pose == None:
      return False

    # move arm to the starting position relative to AR tag
    if not self._preview_traj or not self.goto_part_with_id(id_num):
      rospy.logerr("Fail to move to the starting position for action: " + abbr)
      return False

    # follow the trajectory
    succeed = False
    for i in range(len(self._preview_traj) - 1):
      goal_pose = self._preview_traj[i + 1]
      # move arm relative to the previous pose (use seed), skip the current waypoint if the current action fails
      action_result = self._move_arm_relative(goal_pose.pose, goal_pose.header, seed_state=self._get_seed_state())
      if action_result is not None:
        succeed = True  # the whole action succeeds if at least one pose is reached
      else:
        rospy.logerr("Fail to reach waypoint " + str(i + 1))
    return succeed

  def do_action_with_abbr_smooth(self, abbr, id_num):
    """
      Moves arm to perform the action specified by abbr smoothly.
      Returns true if succeed, false otherwise.
    """
    # preview action
    self.preview_action_with_abbr(abbr, id_num)

    self.freeze_arm()

    # get AR tag information
    tag_pose = self._get_tag_with_id(id_num)
    if tag_pose == None or self._current_pose == None:
      return False

    # move arm to the starting position relative to AR tag
    if not self._preview_traj or not self.goto_part_with_id(id_num):
      rospy.logerr("Fail to move to the starting position for action: " + abbr)
      return False

    # calculate a smooth trajectory passing through all the waypoints and move the arm
    return self._move_arm(None, trajectory_waypoint=self._preview_traj, final_state=True, seed_state=self._get_seed_state())

  def record_action_with_abbr(self, abbr, id_num):
    """
      Records the pose offset named abbr relative to tag, always overwrites the previous entry (if exists).
      Returns true if succeeds, false otherwise.
    """
    # get tag pose
    tag_pose = self._get_tag_with_id(id_num)
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

  def web_app_request_callback(self, msg):
    """
      Parse the request given by the wee application, and call the corresponding functions.
    """
    request_type, request_args = msg.type, msg.args

    print("type: " + request_type)
    print("args: " + str(request_args) + "\n")

    if request_type == "attach":
      return_msg = "SAKE gripper has already attached!"
      if not self._sake_gripper_attached:
        self._publish_server_response(type=request_type, msg="Attaching SAKE gripper...")
        self.attach_sake_gripper()
        self._sake_gripper_attached = True
        return_msg = "SAKE gripper attached"
      self._publish_server_response(type=request_type, status=True, msg=return_msg)

    elif request_type == "remove":
      return_msg = "SAKE gripper has already removed!"
      if self._sake_gripper_attached:
        self._publish_server_response(type=request_type, msg="Removing SAKE gripper...")
        self.remove_sake_gripper()
        self._sake_gripper_attached = False
        return_msg = "SAKE gripper removed"
      self._publish_server_response(type=request_type, status=True, msg=return_msg)

    elif not self._sake_gripper_attached:
      # need to attach SAKE gripper first
      self._publish_server_response(status=True, msg="Please attach SAKE gripper first!")

    else:
      # SAKE gripper has already attached
      if request_type == "record":
        self._publish_server_response(type=request_type, msg="Recording the current scene...")
        if self.update_env(update_octo=bool(request_args[0])):
          # get the list of body parts and actions
          parts = self.get_list()
          parts_info = []
          actions_info = []
          if len(parts):
            for part in parts:
              if part.id in BODY_PARTS and part.id in ACTIONS:
                parts_info.append(str(part.id) + ":" + BODY_PARTS[part.id])
                for action in ACTIONS[part.id]:
                  actions_info.append(str(part.id) + ":" + action + ":" + ABBR[action])
          self._publish_server_response(type="parts", args=parts_info)
          self._publish_server_response(type="actions", status=True, args=actions_info, msg="Scene recorded")
        else:
          self._publish_server_response(type=request_type, status=True, msg="Failed to record the current scene!")

      elif request_type == "prev_id" and len(request_args) == 1:
        id_num = int(request_args[0])  # convert from string to int
        self._publish_server_response(type=request_type, status=True, msg="Previewing " + BODY_PARTS[id_num] + "...")
        self.preview_body_part_with_id(id_num)

      elif request_type == "prev" and len(request_args) == 2:
        abbr, id_num = request_args[0], int(request_args[1])
        waypoints_with_respect_to_tag = self.preview_action_with_abbr(abbr, id_num)
        self._publish_server_response(type=request_type, status=True, args=waypoints_with_respect_to_tag, msg="Previewing action " + abbr + " with respect to body part " + BODY_PARTS[id_num] + "...")

      elif request_type == "highlight" and len(request_args) == 1:
        waypoint_id = int(request_args[0])
        self.highlight_waypoint(self._preview_traj[waypoint_id], WAYPOINT_HIGHLIGHT_COLOR)
        self._publish_server_response(status=True)

      elif request_type == "edit" and len(request_args) == 4:
        waypoint_id, delta_x, delta_y, camera = int(request_args[0]), int(request_args[1]), int(request_args[2]), request_args[3]
        self.edit_waypoint(waypoint_id, delta_x, delta_y, camera)
        self._publish_server_response(status=True)

      elif request_type == "save_edit" or request_type == "cancel_edit":
        if request_type == "save_edit":
          self.modify_traj_in_db(cancel_change=False)
        else:
          self.modify_traj_in_db()  # cancel all the changes
        self._publish_server_response(status=True)

      elif request_type == "reset":
        self._publish_server_response(type=request_type, msg="Resetting...")
        self.reset()
        self._robot_stopped = False
        self._grasp_position_ready = False
        self._do_position_ready = False
        self._do_position_id = -1
        self._publish_server_response(type=request_type, status=True, msg="Done")

      elif not self._robot_stopped: 
        # moveit controller is running
        if request_type == "go" and len(request_args) == 1:
          self._do_position_ready = False
          id_num = int(request_args[0])
          self._publish_server_response(type=request_type, msg="Moving towards body part " + BODY_PARTS[id_num] + "...")
          if self.goto_part_with_id(id_num):
            self._grasp_position_ready = True
            self._do_position_id = id_num
            self._publish_server_response(type=request_type, status=True, msg="Done, ready to grasp")
          else:
            self._publish_server_response(type=request_type, status=True, msg="Fail to move!")

        elif request_type == "grasp" and self._grasp_position_ready and len(request_args) == 1:
          self._publish_server_response(type=request_type, msg="Grasping...")
          if request_args[0] == "h":
            self.do_sake_gripper_action("h_close")
          else:
            self.do_sake_gripper_action("s_close")
          self._grasp_position_ready = False
          self._do_position_ready = True
          self._publish_server_response(type=request_type, status=True, msg="Grasped")

        elif request_type == "relax":
          self._publish_server_response(type=request_type, msg="Relaxing arm...")
          self.relax_arm()
          self._publish_server_response(type=request_type, status=True, msg="Arm relaxed")

        elif request_type == "freeze":
          self._publish_server_response(type=request_type, msg="Freezing arm...")
          self.freeze_arm()
          self._publish_server_response(type=request_type, status=True, msg="Arm froze")

        elif request_type == "do" and len(request_args) > 0:
          return_msg = ""
          if self._do_position_ready:
            # performing mode
            self._publish_server_response(type=request_type, msg="Performing " + request_args[0] + "...")
            if self.do_action_with_abbr(request_args[0], self._do_position_id):
              return_msg = "Action succeeded"
            else:
              return_msg = "Action failed!"
            self._do_position_ready = False
          else:
            return_msg = "Unknown action for body part with ID: " + str(self._do_position_id)
          # always release gripper
          self._publish_server_response(type=request_type, msg="Releasing the gripper...")
          self.do_sake_gripper_action("40 " + self._sake_gripper_effort)
          self._do_position_ready = False
          self._publish_server_response(type=request_type, status=True, msg=return_msg)

        elif request_type == "release":
          self._publish_server_response(type=request_type, msg="Releasing the gripper...")
          self.do_sake_gripper_action("open")
          self._do_position_ready = False
          self._publish_server_response(type=request_type, status=True, msg="Gripper released")

        elif request_type == "stop":
          self._publish_server_response(type=request_type, msg="Stopping the robot...")
          self.relax_arm()
          self._robot_stopped = True
          self._grasp_position_ready = False
          self._do_position_ready = False
          self._do_position_id = -1
          self._publish_server_response(type=request_type, status=True, msg="Robot stopped, please \"RESET\" if you want to continue using it")

        elif request_type == "run" and len(request_args) == 3:
          self.web_app_request_callback(WebAppRequest(type="go", args=[request_args[0]]))
          self.web_app_request_callback(WebAppRequest(type="grasp", args=[request_args[1]]))
          self.web_app_request_callback(WebAppRequest(type="do", args=[request_args[2]]))
          self._publish_server_response(type=request_type, status=True, msg="DONE")

      else: 
        self._publish_server_response(type=request_type, status=True, msg="Invalid command :)")


  def _move_arm_relative(self, ref_pose, ref_header, offset=None, preview_only=False, seed_state=None):
    """ 
      Calculates the coordinate of the goal by adding the offset to the given reference pose, 
      and moves the arm to the goal. If it's only for previewing, returns the goal pose,
      else returns the result of the movement.
    """
    goal_pose = PoseStamped()
    goal_pose.header = ref_header

    if offset is not None:
      # current pose is valid, perform action
      t_mat = self._pose_to_transform(ref_pose)
      # compute the new coordinate
      new_trans = np.dot(t_mat, offset)
      pose = self._transform_to_pose(new_trans)
      goal_pose.pose = pose
    else:
      goal_pose.pose = ref_pose
    
    if preview_only:
      return goal_pose
    else:
      # move to the goal position while avoiding unreasonable trajectories!
      # hard close SAKE gripper again to ensure the limb is grasped
      self.do_sake_gripper_action("h_close")
      # visualize goal pose
      self.highlight_waypoint(goal_pose, ColorRGBA(1.0, 1.0, 0.0, 0.8))
      return goal_pose if self._move_arm(goal_pose, final_state=True, seed_state=seed_state) else None

  def _move_arm(self, goal_pose, trajectory_waypoint=[], final_state=False, seed_state=None):
    """ 
      Moves arm to the specified goal_pose. Returns true if succeed, false otherwise.
    """
    error = None
    if not final_state:
      # simply go to the goal_pose
      error = self._arm.move_to_pose_with_seed(goal_pose, seed_state, [], **self._kwargs)
      # record the current pose because we still have the "do action" step to do
      self._current_pose = goal_pose
    else:
      # go to goal_pose while avoiding unreasonable trajectories!
      if trajectory_waypoint:
        # create an array of waypoints
        ################################################################################################
        waypoints = []
        for tw in trajectory_waypoint:
          waypoints.append(tw.pose)
        # using trajectory waypoints to perform a smooth motion
        plan = self._arm.get_cartesian_path(self._moveit_group, seed_state, waypoints)
        if plan:
          error = self._arm.execute_trajectory(self._moveit_group, plan)
      else:
        # using seed
        error = self._arm.move_to_pose_with_seed(goal_pose, seed_state, [], **self._kwargs)
        if error is not None:
          # planning with seed failed, try without seed 
          # moveit: move group commander
          # check if the pose can be reached in a straight line motion
          plan = self._arm.straight_move_to_pose_check(self._moveit_group, goal_pose)
          if plan:
            error = self._arm.straight_move_to_pose(self._moveit_group, plan)
          else:
            error = 'PLANNING_FAILED'

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

  def _get_tag_with_id(self, id_num):
    """ Returns the AR tag with the given id, returns None if id not found """
    tag_pose = self._reader.get_tag(id_num)
    if tag_pose == None:
      rospy.logerr("AR tag lookup error: Invalid ID# " + str(id_num))
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

  def _get_goto_pose(self, ar_pose):
    """
      Calculates the grasp pose of gripper given the AR tag pose, returns the gripper pose.
    """
    grasp_offset = self._db.get("GRASP")
    goal_pose = self._move_arm_relative(ar_pose.pose.pose, ar_pose.header, offset=grasp_offset, preview_only=True)
    self.highlight_waypoint(goal_pose, WAYPOINT_HIGHLIGHT_COLOR)
    return goal_pose

  def _save_traj_to_db(self, abbr, id_num):
    """
      Checks bag file for the trajectory with given abbr, calculate waypoints and save those points to the database.
      Returns the calculated waypoints if succeed, none otherwise.
    """
    # check bag files for the trajectory
    bag_file_path = os.path.join(self._bag_file_dir, abbr.lower() + '.bag')
    bag = rosbag.Bag(bag_file_path)
    waypoints = []
    prev_msg = []
    # get the trajectory from bag file
    for topic, msg, t in bag.read_messages(topics=['/joint_states']):
      joint_state = list(msg.position[6:13])
      if len(joint_state) != 0 and (len(prev_msg) == 0 or np.abs(np.sum(np.subtract(joint_state, prev_msg))) > rospy.get_param("arm_traj_threshold")):
        prev_msg = joint_state
        # use forward kinematics to find the wrist position
        point = self._arm.compute_fk(msg)
        if point:
          # add the result position
          waypoints.append(point[0])
    bag.close()

    if len(waypoints) < 2:  # empty trajectory
      rospy.logerr("Empty trajectory for action: " + abbr)
      return None
    
    # add the result position to database
    self._db.add(abbr, waypoints)
    self._db.save()
    return waypoints

  def _publish_server_response(self, type="", status=False, args=[], msg=""):
    """ Publishes the server response message, and prints the message in console if needed. """
    if rospy.get_param("console_output"):
      print(msg)
    self._web_app_response_pub.publish(WebAppResponse(type=type, status=status, args=args, msg=msg))

  def _compute_pose_by_delta(self, current_pose, delta_x, delta_y, camera):
    """ 
      Computes and returns the new pose with respect to base_link after applying 
      delta_x and delta_y to the current pose in the specified camera view.
    """
    x_distance, y_distance = dpx_to_distance(delta_x, delta_y, camera, current_pose, True)
    return delta_modified_stamped_pose(x_distance, y_distance, camera, current_pose)

  def _get_seed_state(self):
    """ Returns the current arm joint state as the seed used in motion planning. """
    seed_state = JointState()
    seed_state.name = self._arm_joints.names()
    seed_state.position = self._arm_joints.values()
    return seed_state

  def _set_sake_gripper_action_status(self):
    """ This is the callback of sake gripper status. """
    self._sake_gripper_action_finished = True