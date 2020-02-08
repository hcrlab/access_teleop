#! /usr/bin/env python


import rospy
import math
from pprint import pprint
from limb_pbd_server import PbdServer, BODY_PARTS, ACTIONS, ABBR
from shared_teleop_functions_and_vars import wait_for_time


def print_usage():
  print("Commands:")
  print("  attach: close Fetch's gripper to hold SAKE gripper")
  print("  remove: open Fetch's gripper to remove SAKE gripper\n")
  print("  record: record the current scene and update body parts known to the robot")
  print("    -o: update octomap as well")
  print("  parts: show a list of body parts (eg: right lower leg (ID#))")
  print("  actions: show a list of available actions (eg: leg abduction (ABBREVIATION))")
  print("  prev_id <ID#>: preview the body part with ID#")
  print("  prev <ABBR> <ID#>: preview the trajectory of action ABBR with respect to body part ID#\n")
  print("  go <ID#>: move the gripper to a place 10cm from the body part specified by ID#")
  print("  grasp: if followed by \"go <ID#>\", move the gripper down to grasp the body part with these options (default: -s)")
  print("    -h: hard close gripper")
  print("    -s: soft close gripper")
  print("    <PERCENTAGE_OPEN> <EFFORT>")
  print("  relax: relax the robot arm")
  print("  freeze: freeze the robot arm")
  print("  do <ABBR>: perform the action specified by ABBR")
  print("    -s: do the action smoothly")
  print("    -r: if the robot arm is relaxed, save the current pose as ABBR")
  print("  open: if the gripper is closed, open it")
  print("  reset: move Fetch's arm to its initial position, open the gripper if it's closed\n")
  print("  stop: emergency stop\n")
  print("  help: print program usage\n")
  print("  db_list: list entries in database")
  print("  db_print <ABBR>: print values associated with the entry")
  print("  db_delete <ABBR>: delete the entry in database")
  print("  quit: shutdown the program\n")

def main():
  print("\n***************** LIMB PBD *****************")

  rospy.init_node('limb_pbd')
  wait_for_time()

  server = PbdServer()
  server.setup()
  rospy.sleep(0.5)

  # shutdown handler
  def handle_shutdown():
    server.shutdown()
  
  rospy.on_shutdown(handle_shutdown)
  
  # variables representing the program state
  sake_gripper_attached = False
  robot_stopped = False
  grasp_position_ready = False
  do_position_ready = False
  do_position_id = -1

  print_usage()

  # main loop
  while not rospy.is_shutdown():
    command = raw_input("> ")

    if command[:4] == "help":
      print_usage()

    elif command[:4] == "quit":
      print("Exitting...")
      break

    elif command[:7] == "db_list":
      print("Entries in the database:")
      list = server.get_db_list()
      print(list)

    elif command[:8] == "db_print" and len(command) > 9:
      print("Printing: " + command[9:] + "...")
      values = server.get_db_entry(command[9:])
      if values:
        id_num = 0
        for v in values:
          print(str(id_num) + ": ")
          print(v)
          id_num += 1
      else:
        print("Entry not found!")

    elif command[:9] == "db_delete" and len(command) > 10:
      print("Deleting: " + command[10:] + "...")
      server.delete_db_entry(command[10:])

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
          result = False
          if len(command) > 8 and command[8] == "o":
            result = server.update_env()
          else:
            result = server.update_env(update_octo=False)
          if result:
            print("Scene recorded")
          else:
            print("Failed to record the current scene!")

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

        elif command[:7] == "prev_id" and len(command) > 8:
          try:
            id_num = int(command[8:])  # convert from string to int
            if id_num not in BODY_PARTS:
              print("Given number is invalid!")
            else:
              print("Previewing body part with id " + command[8:] + "...")
              server.preview_body_part_with_id(id_num)
          except ValueError:
            print("Please enter an integer!")

        elif command[:4] == "prev" and len(command) > 5:
          prev_args = command.split()
          if len(prev_args) == 3:
            abbr = prev_args[1]
            id_num = prev_args[2]
            print("Previewing action " + abbr + " with respect to body part " + id_num + "...")
            server.preview_action_with_abbr(abbr, int(id_num))
          else:
            print("Not enough arguments")

        elif command[:5] == "reset":
          print("Resetting...")
          server.reset()
          robot_stopped = False
          grasp_position_ready = False
          do_position_ready = False
          do_position_id = -1

        elif not robot_stopped: 
          # moveit controller is running
          if command[:2] == "go" and len(command) > 3:
            do_position_ready = False
            print("Moving towards body part #" + command[3:] + "...")
            try:
              id_num = int(command[3:])  # convert from string to int
              if id_num not in BODY_PARTS:
                print("Given number is invalid!")
              elif server.goto_part_with_id(id_num):  # given id# is valid
                print("Done, ready to grasp")
                grasp_position_ready = True
                do_position_id = id_num
              else:
                print("Fail to move!")
            except ValueError:
              print("Please enter an integer!")

          elif command[:5] == "grasp" and grasp_position_ready:
            print("Grasping...")
            args = command[6:].split(" ")
            if len(args) == 1 and args[0] == "-h":  # hard close
                server.do_sake_gripper_action("h_close")                
            else if len(args) == 2:  # percentage_close effort
              try:
                percentage_open = int(args[0])
                effort = int(args[1])
              except ValueError:
                print("Please enter an integer!")
            else: 
              server.do_sake_gripper_action("s_close")
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
            if command[:5] == "do -s" and len(command) > 6 and command[6:] in ABBR.values() and do_position_ready:
              # performing mode (smooth)
              print("Performing " + command[6:] + "... (smooth)")
              if server.do_action_with_abbr_smooth(command[6:], do_position_id):
                print("Action succeed")
              else:
                print("Action failed!")
              do_position_ready = False
            elif len(command) > 3 and command[3:] in ABBR.values() and do_position_ready:
              # performing mode
              print("Performing " + command[3:] + "...")
              if server.do_action_with_abbr(command[3:], do_position_id):
                print("Action succeed")
              else:
                print("Action failed!")
              do_position_ready = False
            elif command[:5] == "do -r" and len(command) > 6: # and command[6:] in ABBR.values():
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
            server.do_sake_gripper_action("open")
            do_position_ready = False

          elif command[:7] == "open":
            print("Opening the gripper...")
            server.do_sake_gripper_action("open")
            do_position_ready = False

          elif command[:4] == "stop":
            print("Stopping the robot...")
            server.relax_arm()
            robot_stopped = True
            grasp_position_ready = False
            do_position_ready = False
            do_position_id = -1
            print("Robot stopped, please \"reset\" if you want to continue using it")

        else: 
          print("Invalid command :)")


if __name__ == "__main__":
  main()
