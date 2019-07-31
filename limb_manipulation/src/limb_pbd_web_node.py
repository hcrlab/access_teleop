#! /usr/bin/env python

import rospy
from pprint import pprint
from limb_pbd_server import wait_for_time, PbdServer

def main():
  print("\n**************** LIMB PBD *****************")

  rospy.init_node('limb_pbd_web_node')
  wait_for_time()

  server = PbdServer()
  server.setup()
  rospy.sleep(0.5)

  # shutdown handler
  def handle_shutdown():
    server.shutdown()
  
  rospy.on_shutdown(handle_shutdown)
  rospy.spin()

if __name__ == "__main__":
  main()
