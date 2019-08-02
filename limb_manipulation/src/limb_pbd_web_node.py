#! /usr/bin/env python

import rospy
from pprint import pprint
from limb_pbd_server import wait_for_time, PbdServer
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster
import camera_info_messages
from shared_teleop_functions_and_vars import publish_camera_transforms, publish_camera_info, camera_names

def main():
  print("\n**************** LIMB PBD *****************")

  rospy.init_node('limb_pbd_web_node')
  wait_for_time()

  # server
  server = PbdServer()
  server.setup()

  # camera views
  camera_model = PinholeCameraModel()
  tb = TransformBroadcaster()
  info_pubs = []
  for camera_name in camera_names:
    info_pubs.append([camera_name,
                      rospy.Publisher(camera_name + '/camera_info', camera_info_messages.CameraInfo, queue_size=1)])

  rospy.sleep(0.5)

  # shutdown handler
  def handle_shutdown():
    server.shutdown()
  
  rospy.on_shutdown(handle_shutdown)
  
  rate = rospy.Rate(200)
  while not rospy.is_shutdown():
    publish_camera_transforms(tb)
    publish_camera_info(info_pubs)
    rate.sleep()

if __name__ == "__main__":
  main()
