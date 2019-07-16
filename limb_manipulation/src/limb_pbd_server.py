#! /usr/bin/env python

import rospy
import math
from pprint import pprint
import fetch_api
from std_msgs.msg import String, Header, ColorRGBA, Bool
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers
from limb_manipulation_msgs.msg import EzgripperAccess
from sake_gripper import SakeEzGripper

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass


class ArTagReader(object):
  def __init__(self):
    self.markers = []  # list of markers (update in real time)

  def callback(self, msg):
    self.markers = msg.markers

  def get_tag(self, tag):
    """ Get the marker with id# == tag """
    for marker in self.markers:
      if marker.id == int(tag):
        result = PoseStamped()
        result.pose = marker.pose
        result.header = marker.header
        return result
  
  def get_list(self):
    return self.markers


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
    self._move_group = MoveGroupCommander("arm")
    # AR tag reader
    self._reader = ArTagReader()
    self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self._reader.callback)
    # controls of SAKE gripper
    self._sake_gripper = SakeEzGripper(rospy.get_param('~grippers'))
    self._sake_gripper.start()
    self._sake_gripper_pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    
    rospy.sleep(0.5)


  def setup(self):
    # robot set up
    self._torso.set_height(0.4)
    self._head.pan_tilt(0, math.pi / 3)

    # collision detection

    # move arm to the initial position (with collision detection)

    # calibrate SAKE gripper
    print("Calibrating SAKE gripper, please wait...")
    self.calibrate_sake_gripper()

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
