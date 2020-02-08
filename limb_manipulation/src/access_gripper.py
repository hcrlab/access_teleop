#!/usr/bin/python

import rospy
from ezgripper_libs.ezgripper_interface import EZGripper
from limb_manipulation_msgs.msg import EzgripperAccess

class EZGripperAccess(object):
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
		rospy.Subscriber("/ezgripper_access", EzgripperAccess, ezgripper_access.access_callback)
		self.pub = rospy.Publisher("/ezgripper_access_status", EzgripperAccess, queue_size=1)

	def access_callback(self, data):
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

			if len(data.type) > 2:
				commands = data.type.split(" ")  # [percentage open, effort]
				if len(commands) == 2:
					open_percentage = int(commands[0])
					effort = int(commands[1])
					# 0% (close) ---> 100% (open)
					# >0 (min effort) ---> 100 (max effort)
					if 0 <= open_percentage <= 100 and 0 < effort <= 100:
						gripper.goto_position(open_percentage, effort)
						self.last_command_end_time = rospy.get_rostime()

			self.pub.publish(data)

if __name__ == "__main__":
	rospy.init_node("access_ezgripper")
	rospy.sleep(0.5)

	gripper_names = rospy.get_param('~grippers')
	ezgripper_access = EZGripperAccess(gripper_names)
	ezgripper_access.start()

	rospy.sleep(0.5)
	rospy.spin()


