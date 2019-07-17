#! /usr/bin/env python

import rospy
import math
from pprint import pprint
import fetch_api
from std_msgs.msg import String, Header, ColorRGBA, Bool
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
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
    """ Get the marker with id# == tag """
    for marker in self.saved_markers:
      if marker.id == int(tag):
        result = PoseStamped()
        result.pose = marker.pose
        result.header = marker.header
        return result
    return None
  
  def get_list(self):
    return self.saved_markers


class PbdServer():
  """
  Command List:
    parts: show a list of body parts (eg: right lower leg (ID#))
    actions: show a list of available actions (eg: leg abduction (ABBREVIATION))

    go ID#: move the gripper to a place 10cm from the body part specified by ID#
    grasp: if followed by "go ID#", move the gripper down to grasp the body part
    release: if the gripper is closed, open it
    reset: move Fetch's arm to its initial position, open the gripper if it's closed

    do ABBR: perform the action at once
    stop: emergency stop

    help: print program usage
  """
  def __init__(self):
    # controls of Fetch
    self._arm = fetch_api.Arm()
    self._torso = fetch_api.Torso()
    self._head = fetch_api.Head()
    self._base = fetch_api.Base()
    self._fetch_gripper = fetch_api.Gripper()
    # AR tag reader
    self._reader = ArTagReader()
    self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self._reader.callback)
    # publisher for controls of SAKE gripper
    self._sake_gripper_pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    # motion planning
    self._planning_scene = PlanningSceneInterface('base_link')
    self._planning_scene.clear()

    rospy.sleep(0.5)


  def setup(self):
    # robot set up
    self._torso.set_height(0.1)
    self._head.pan_tilt(0, math.pi / 3)

    # attach SAKE gripper to Fetch's gripper
    self._planning_scene.removeAttachedObject('sake')
    sake_gripper_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    sake_gripper_mesh_file = "../ezgripper_driver/meshes/ezgripper_gen2/SAKE_Palm_Dual_Gen2.stl"
    # name	Name of the object
    # pose	A geometry_msgs/Pose for the object
    # filename	The mesh file to load
    # wait	When true, we wait for planning scene to actually update, this provides immunity against lost messages.
    self._planning_scene.attachMesh('sake', sake_gripper_pose, sake_gripper_mesh_file, True)

    # collision detection


    # move arm to the initial position (with collision detection)

    # calibrate SAKE gripper
    print("Calibrating SAKE gripper, please wait...")
    self.calibrate_sake_gripper()

  def shutdown(self):
    # robot shut down
    self._planning_scene.removeAttachedObject('sake')
    self._planning_scene.clear()
    self._arm.cancel_all_goals()

  def update_list(self):
    """ Updates the list of markers """
    self._reader.update()
  
  def get_list(self):
    """ Returns a list of AR tags recognized by the robot """
    return self._reader.get_list()

  def calibrate_sake_gripper(self):
    self._sake_gripper_pub.publish(EzgripperAccess(type="calibrate"))

  def hard_close_sake_gripper(self):
    self._sake_gripper_pub.publish(EzgripperAccess(type="h_close"))

  def soft_close_sake_gripper(self):
    self._sake_gripper_pub.publish(EzgripperAccess(type="s_close"))
  
  def open_sake_gripper(self):
    self._sake_gripper_pub.publish(EzgripperAccess(type="open"))

  def goto_part_with_id(self, id):
    pose = self._reader.get_tag(id)
    if pose is not None:
      # found marker, move towards it

      # 10 cm above the marker


      gripper_oc = OrientationConstraint()
      gripper_oc.header.frame_id = 'base_link'
      gripper_oc.link_name = 'wrist_roll_link'
      gripper_oc.orientation.z = -0.707
      gripper_oc.orientation.w = 0.707
      gripper_oc.absolute_x_axis_tolerance = 0.1
      gripper_oc.absolute_y_axis_tolerance = 0.1
      gripper_oc.absolute_z_axis_tolerance = 3.14
      gripper_oc.weight = 1.0

      kwargs = {
        'allowed_planning_time': 10,
        'execution_timeout': 30,
        'group_name': 'arm',
        'num_planning_attempts': 10,
        'orientation_constraint': gripper_oc,
        'replan': True,
        'replan_attempts': 5,
        'tolerance': 0.001
      }

      error = self._arm.move_to_pose(pose, **kwargs)
      if error is not None:
        self._arm.cancel_all_goals()
        rospy.logerr('Fail to move: {}'.format(error))
        return False
      # success
      return True
    else:
      # marker with id is not found
      return False
