#!/usr/bin/python

import rospy
from ezgripper_libs.ezgripper_interface import EZGripper
from limb_manipulation_msgs.msg import EzgripperAccess
import time

class SakeEzGripper(object):
    """
        Controls for SAKE gripper
    """

    def __init__(self, gripper_names):
        self.ezgripper_left = EZGripper(gripper_names[0])

        # For multiple grippers
        # if len(gripper_names) > 1:
        #     self.ezgripper_right = EZGripper(gripper_names[1])
        # else:
        #     self.ezgripper_right = None

        self.last_command_end_time = rospy.get_rostime()

    def start(self):
        rospy.Subscriber("/ezgripper_access", EzgripperAccess, self.callback)

    def callback(self, data):
        if (rospy.get_rostime() - self.last_command_end_time).to_sec() > 0.2:
            # This check should flush all messages accumulated during command execution
            # and avoid executing it again.

            gripper = self.ezgripper_left
        
            if data.type == "h_close":    # hard close
                gripper.hard_close()
                self.last_command_end_time = rospy.get_rostime()

            if data.type == "s_close":    # soft close
                gripper.soft_close()
                self.last_command_end_time = rospy.get_rostime()
                
            if data.type == "open":       # open
                gripper.open()
                self.last_command_end_time = rospy.get_rostime()
    
            if data.type == "release":    # release
                gripper.release()
                self.last_command_end_time = rospy.get_rostime()
    
            if data.type == "calibrate":  # calibrate
                gripper.calibrate()
                self.last_command_end_time = rospy.get_rostime()


if __name__ == "__main__":
    rospy.init_node("sake_gripper")
    rospy.sleep(0.5)

    gripper_names = rospy.get_param('~grippers')
    ezgripper_access = SakeEzGripper(gripper_names)
    ezgripper_access.start()

    rospy.sleep(0.5)
    rospy.spin()
