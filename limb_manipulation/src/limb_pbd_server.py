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

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass


class ArTagReader(object):
  def __init__(self):
    self.markers = []
    self.savedMarkers = []

  def callback(self, msg):
    self.markers = msg.markers

  def update(self):
    self.savedMarkers = self.markers

  def getTag(self, tag):
    for marker in self.savedMarkers:
      if marker.id == int(tag):
        result = PoseStamped()
        result.pose = marker.pose
        result.header = marker.header
        return result


class PbdServer():
  def __init__(self):
    self._arm = fetch_api.Arm()
    self._torso = fetch_api.Torso()
    self._head = fetch_api.Head()
    self._base = fetch_api.Base()
    self._gripper = fetch_api.Gripper()
    self._move_group = MoveGroupCommander("arm")

  def setup(self):
    # robot set up
    self._torso.set_height(0.4)
    self._head.pan_tilt(0, math.pi / 3)

    # collision detection


    # move arm to the initial position
