#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from limb_pbd_server import wait_for_time, PbdServer, BODY_PARTS, ACTIONS, ABBR


def print_usage():
  print("Commands:")
  print("  attach: close Fetch's gripper to hold SAKE gripper")
  print("  remove: open Fetch's gripper to remove SAKE gripper\n")
  print("  record: record the current scene and update body parts known to the robot")
  print("  parts: show a list of body parts (eg: right lower leg (ID#))")
  print("  actions: show a list of available actions (eg: leg abduction (ABBREVIATION))\n")
  print("  go ID#: move the gripper to a place 10cm from the body part specified by ID#")
  print("  grasp: if followed by \"go ID#\", move the gripper down to grasp the body part with these options (default: -s)")
  print("    -h: hard close gripper")
  print("    -s: soft close gripper")
  print("  relax: relax the robot arm")
  print("  freeze: freeze the robot arm")
  print("  do ABBR: perform the action specified by ABBR")
  print("    -r: if the robot arm is relaxed, record the pose named ABBR")
  print("  release: if the gripper is closed, open it")
  print("  reset: move Fetch's arm to its initial position, open the gripper if it's closed\n")
  print("  stop: emergency stop\n")
  print("  help: print program usage")
  print("  quit: shutdown the program\n")

def main():
  print("\n**************** LIMB PBD *****************")

  rospy.init_node('limb_pbd')
  wait_for_time()

  server = PbdServer()
  server.setup()
  rospy.sleep(0.5)

  # shutdown handler
  def handle_shutdown():
    server.shutdown()
  
  rospy.on_shutdown(handle_shutdown)
  
  sake_gripper_attached = False
  grasp_position_ready = False
  do_position_ready = False
  do_position_id = -1

  print("\nThe program is ready to use :-)\n")
  print_usage()

  while not rospy.is_shutdown():
    command = raw_input("> ")

    if command[:4] == "help":
      print_usage()

    elif command[:4] == "quit":
      print("Exitting...")
      break

    else:
      if command[:6] == "attach":
        if not sake_gripper_attached:
          print("Attaching SAKE gripper...")
          server.attach_sake_gripper()
          sake_gripper_attached = True
        else:
          print("SAKE gripper has already attached!")

      elif command[:6] == "remove":
        if sake_gripper_attached:
          print("Removing SAKE gripper...")
          server.remove_sake_gripper()
          sake_gripper_attached = False
        else:
          print("SAKE gripper has already removed!")

      elif not sake_gripper_attached:
        # need to attach SAKE gripper first
        print("Please attach SAKE gripper first!")
        
      else:
        # SAKE gripper has already attached
        if command[:6] == "record":
          print("Recording the current scene...")
          server.update_list()

        elif command[:5] == "parts":
          parts = server.get_list()
          if len(parts):
            print("Below are the body parts recognized by the robot:")
            for part in parts:
              if part.id in BODY_PARTS:
                print(BODY_PARTS[part.id] + " ID: " + str(part.id))
              else:
                print("Unknown part ID: " + str(part.id))
          else:
            print("No part found")

        elif command[:7] == "actions":
          parts = server.get_list()
          if len(parts):
            print("Below is the list of available actions:")
            for part in parts:
              if part.id in ACTIONS:
                print("Part ID " + str(part.id) + ": " + BODY_PARTS[part.id])
                for action in ACTIONS[part.id]:
                  print("\t" + action + ", abbreviation: " + ABBR[action])
              else:
                print("Part ID " + str(part.id) + " has no action available")
          else:
            print("No action found")

        elif command[:5] == "reset":
          print("Resetting...")
          server.reset()
          grasp_position_ready = False
          do_position_ready = False
          do_position_id = -1

        elif command[:2] == "go" and len(command) > 3:
          do_position_ready = False
          print("Moving towards body part #" + command[3:] + "...")
          try:
            id = int(command[3:])  # convert from string to int
            if id not in BODY_PARTS:
              print("Given number is invalid!")
            elif server.goto_part_with_id(id):  # given id# is valid
              print("Done, ready to grasp")
              grasp_position_ready = True
              do_position_id = id
            else:
              print("Fail to move!")
          except ValueError:
            print("Please enter an integer!")

        elif command[:5] == "grasp" and grasp_position_ready:
          print("Grasping...")
          if len(command) > 7 and command[7] == "h":
            server.hard_close_sake_gripper()
          else:
            server.soft_close_sake_gripper()
          grasp_position_ready = False
          do_position_ready = True

        elif command[:5] == "relax":
          print("Relaxing arm...")
          server.relax_arm()
          print("Arm relaxed, please move the arm to the goal position, and use \"do -r ABBR\" to record the pose")

        elif command[:6] == "freeze":
          print("Freezing arm...")
          server.freeze_arm()

        elif command[:2] == "do" and len(command) > 3:
          if len(command) > 3 and command[3:] in ABBR.values() and do_position_ready:
            # performing mode
            print("Performing " + command[3:] + "...")
            if server.do_action_with_abbr(command[3:], do_position_id):
              print("Action succeed")
            else:
              print("Action failed!")
            do_position_ready = False
          elif command[:5] == "do -r" and len(command) > 6 and command[6:] in ABBR.values():
            # recording mode
            print("Recording " + command[6:] + ", please don't move the robot arm...")
            if server.record_action_with_abbr(command[6:], do_position_id):
              do_position_ready = False
              # freeze the arm
              print("Action recorded, freezing the robot arm...")
            else:
              print("Fail to record this action! Freezing robot arm...")
            server.freeze_arm()
          else:
            print("Unknown action for body part with ID: " + str(do_position_id))
          # always release gripper
          print("Releasing the gripper...")
          server.open_sake_gripper()
          do_position_ready = False

        elif command[:7] == "release":
          print("Releasing the gripper...")
          server.open_sake_gripper()
          do_position_ready = False

        elif command[:4] == "stop":
          print("Stopping the robot...")
          ####################################################################TODO###############################
          grasp_position_ready = False
          do_position_ready = False
          do_position_id = -1

        else: 
          print("Invalid command :)")


if __name__ == "__main__":
  main()
