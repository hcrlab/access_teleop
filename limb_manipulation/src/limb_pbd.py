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

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass


def main():
    rospy.init_node('limb_pbd')
    wait_for_time()

    torso = fetch_api.Torso()
    head = fetch_api.Head()


    torso.set_height(0.4)
    head.pan_tilt(0, math.pi / 3)

    rospy.spin()


if __name__ == "__main__":
    main()
