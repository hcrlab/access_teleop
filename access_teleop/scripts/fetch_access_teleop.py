#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX, PXAndTheta, Theta, TaskType, HeadZ
import fetch_api
import camera_info_messages
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from shared_teleop_functions_and_vars import wait_for_time, quat_array_to_quat, publish_camera_transforms, publish_camera_info, \
    publish_gripper_pixels, dpx_to_distance, delta_modified_stamped_pose, \
    absolute_modified_stamped_pose, add_marker, addSetback, orientation_mapping, orientation_sign_mapping, camera_names

HEAD_POSE = [1.7, -0.1, 0.3]

class MoveByDelta(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback, queue_size=1)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = delta_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible: #This check will prevent some edge poses, but will also save time
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                print("We got there!")


class MoveByAbsolute(object):

    def __init__(self, arm, move_group, status_pub):
        self._arm = arm
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=2)
        self._status_pub = status_pub

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback, queue_size=1)

    def absolute_callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        #add_marker(x_distance, y_distance, ps, data.camera_name, self)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible: #This check will prevent some edge poses, but will also save time
            self._status_pub.publish("moving")
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                self._status_pub.publish("arrived")
        else:
            self._status_pub.publish("unreachable")


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        #rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group
        self.SETBACK = 0.15

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        #rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        ps3 = addSetback(self.SETBACK, data.theta, data.camera_name, ps2)
        error = self._arm.move_to_pose(ps3, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class Orient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/orient', Theta, self.orient_callback, queue_size=1)

    def orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        #rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] += data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        error = self._arm.move_to_pose(ps, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class WristRoll(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/wrist_roll', Theta, self.wrist_roll_callback, queue_size=1)

    def wrist_roll_callback(self, data):
        self._move_group.clear_pose_targets()
        arm_values = self._move_group.get_current_joint_values()
        arm_values[6] += data.theta
        self._move_group.set_joint_value_target(arm_values)
        self._move_group.go()


# Added by Xinyi for switching the task in action test
class BaseSwitchTask(object):
    def __init__(self, base):
        self._base = base

    def start(self):
        rospy.Subscriber('/access_teleop/task_type', TaskType, self.base_switch_task_callback, queue_size=1)

    def base_switch_task_callback(self, data):
        if data.task_type is 0:  # go to previous task
            self._base.turn(math.pi / 2)
            self._base.align_with_y_axis_pos()

            self._base.go_forward(0.95, 0.3)
            self._base.stop()
            rospy.sleep(0.03)

            self._base.turn(-math.pi / 2)
            self._base.align_with_x_axis_pos()
        else:  # go to next task
            self._base.turn(-math.pi / 2)
            self._base.align_with_y_axis_neg()

            self._base.go_forward(0.95, 0.3)
            self._base.stop()
            rospy.sleep(0.03)

            self._base.turn(math.pi / 2)
            self._base.align_with_x_axis_pos()

# Added by Xinyi for tilting robot's head
class HeadTilt(object):
    def __init__(self, head):
        self._head = head

    def start(self):
        rospy.Subscriber('/access_teleop/head_z', HeadZ, self.head_tilt_callback, queue_size=1)

    def head_tilt_callback(self, data):
        HEAD_POSE[2] += data.tilt
        self._head.look_at("base_link", HEAD_POSE[0], HEAD_POSE[1], HEAD_POSE[2])

        


def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    # set up controls for each part
    arm = fetch_api.Arm()
    base = fetch_api.Base()
    torso = fetch_api.Torso()
    head = fetch_api.Head()

    move_group = MoveGroupCommander("arm")


    status_publisher = rospy.Publisher('/access_teleop/arm_status', String, queue_size=1)
    gripper_publisher = rospy.Publisher('/access_teleop/gripper_pixels', PX, queue_size=1)

    info_pubs = []
    for camera_name in camera_names:
        info_pubs.append([camera_name,
                          rospy.Publisher(camera_name + '/camera_info', camera_info_messages.CameraInfo, queue_size=10)])

    # debug: visualize camera positions
    camera_vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)


    tb = TransformBroadcaster()

    camera_model = PinholeCameraModel()

    move_by_delta = MoveByDelta(arm, move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(arm, move_group, status_publisher)

    move_by_absolute.start()

    move_and_orient = MoveAndOrient(arm, move_group)
    move_and_orient.start()

    orient = Orient(arm, move_group)
    orient.start()

    # Added by Xinyi
    wrist_roll = WristRoll(arm, move_group)
    wrist_roll.start()

    base_switch_task = BaseSwitchTask(base)
    base_switch_task.start()

    head_tilt = HeadTilt(head)
    head_tilt.start()

    rospy.sleep(0.5)

    # Raise the torso to allow arm movement
    # torso.set_height(fetch_api.Torso.MAX_HEIGHT)
    torso.set_height(fetch_api.Torso.MAX_HEIGHT) # 0.5

    # Set arm joints
    # INITIAL_POSES = [0.0, 1.25, 0.0, -2.5, 0.0, 1.5, 0.0]
    INITIAL_POSES = [1.0, 1.25, 1.0, -2.5, -0.3, 1.2, 0.0]
    # ---> gripper
    # b: blue joint
    # g: gray joint
    # order: [b, g, b, g, b, g, b]
    # samples: 
    # [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]
    # [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
    # [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
    # [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0]
    # [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
    # [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
    # [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]
    arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSES))


    # Set the base position
    # OPTION 1: Start from origin
    # move Fetch from the origin to the table
    # base.go_forward(1.6, 0.5)  # value (distance in meters)
    # base.turn(math.pi / 3)     # value (angle in degrees)
    # base.go_forward(3.3, 0.5)  # value (distance in meters)
    # base.turn(-math.pi / 3)

    # OPTION 2: Start right in front of the table
    base.align_with_x_axis_pos()
    base.go_forward(0.3, 0.5)


    # Adjust the head to look at the table
    # head.pan_tilt(0, math.pi / 2)
    head.look_at("base_link", HEAD_POSE[0], HEAD_POSE[1], HEAD_POSE[2])
    
    # (end)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_camera_transforms(tb, camera_vis_pub)
        publish_camera_info(info_pubs)
        publish_gripper_pixels(camera_model, move_group, gripper_publisher)
        rate.sleep()


if __name__ == "__main__":
    main()
