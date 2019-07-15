#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from limb_pbd_server import PbdServer

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass

def print_usage():
  print("Commands:")
  print("\tparts: show a list of body parts (eg: right lower leg (ID#))")
  print("\tactions: show a list of available actions (eg: leg abduction (ABBREVIATION))")
  print("\n")
  print("\tgo ID#: move the gripper to a place 10cm from the body part specified by ID#")
  print("\tgrasp: if followed by \"go ID#\", move the gripper down to grasp the body part")
  print("\trelease: if the gripper is closed, open it")
  print("\treset: move Fetch's arm to its initial position, open the gripper if it's closed")
  print("\n")
  print("\tdo ABBR: perform the action at once")
  print("\tstop: emergency stop")
  print("\n")
  print("\thelp: print program usage")

def main():
  rospy.init_node('limb_pbd')
  wait_for_time()

  server = PbdServer()
  server.setup()
  print("******** LIMB PBD ********")
  grasp_position_ready = False

  print_usage()
  while (True):
    command = raw_input("> ")

    if command[:4] == "help":
      print_usage()
    elif command[:5] == "parts":
      print("Below are the body parts recognized by the robot:")
      # 
    elif command[:7] == "actions":
      print("Below is the list of available actions:")
      # 
    elif command[:5] == "reset":
      print("Resetting...")
      # reset
      # grasp_position_ready = False

    elif command[:2] == "go" and len(command) > 3:
      print("Moving towards body part #" + command[3:])
      # check if the given id is valid, if valid: grasp_position_ready = True
    elif command[:5] == "grasp" and grasp_position_ready:
      print("Grasping, please don't move!")
      # open gripper, then grasp
      # grasp_position_ready = False
    elif command[:7] == "release":
      print("Releasing the gripper...")
      # 
    elif command[:2] == "do" and len(command) > 3:
      print("Performing " + command[3:])
      # reset
      # check if the action fails
    elif command[:4] == "stop":
      print("Stopping the robot...")
      # grasp_position_ready = False
    else: 
      print("Invalid command :)")

  rospy.spin()


if __name__ == "__main__":
  main()
