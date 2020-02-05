#! /usr/bin/env python

import rospy
import fetch_api
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import OrientationConstraint

def wait_for_time():
  """
    Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass


def main():
  rospy.init_node('coke_can_node')
  wait_for_time()

  # controls of Fetch
  arm = fetch_api.Arm()
  arm_joints = fetch_api.ArmJoints()
  
  rospy.sleep(0.5)

  pose = PoseStamped()
  pose.header.frame_id = 'base_link'
  pose.pose.position.x = 0.8
  pose.pose.position.y = 0
  pose.pose.position.z = 0.75
  pose.pose.orientation.w = 1

  kwargs = {
      'allowed_planning_time': 100,
      'execution_timeout': 100,
      'num_planning_attempts': 1,
      'replan_attempts': 5,
      'replan': True,
      'orientation_constraint': None
  }

  error = arm.move_to_pose(pose, **kwargs)
  if error is not None:
      rospy.logerr('Pose failed: {}'.format(error))
  else:
      rospy.loginfo('Pose succeeded')
      print "Arm moved!"
  

if __name__ == "__main__":
  main()
