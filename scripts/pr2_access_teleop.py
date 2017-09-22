#! /usr/bin/env python

import rospy
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX
import camera_info_messages
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from shared_teleop_functions_and_vars import wait_for_time, quat_array_to_quat, publish_camera_transforms, publish_camera_info,\
    publish_gripper_pixels, getCameraDistances, dpx_to_distance, delta_modified_stamped_pose,\
    absolute_modified_stamped_pose, add_marker, addSetback, pr2_move_group_name, camera_names


class MoveByDelta(object):
    def __init__(self, move_group):
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback, queue_size=1)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = delta_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        self._move_group.set_pose_target(ps2)
        self._move_group.go(wait=False)


class MoveByAbsolute(object):
    def __init__(self, move_group):
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=1)

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback, queue_size=1)

    def absolute_callback(self, data):
        print("We got the pixel with x of " + str(data.pixel_x) + " and y of " + str(data.pixel_y))
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        self._move_group.set_pose_target(ps2)
        self._move_group.go(wait=False)

        #add_marker(x_distance, y_distance, ps, data.camera_name, self)


def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    print("Don't forget to launch the move group server with roslaunch pr2_moveit_config move_group.launch")
    move_group = MoveGroupCommander(pr2_move_group_name)

    gripper_publisher = rospy.Publisher('/access_teleop/gripper_pixels', PX, queue_size=1)

    camera_model = PinholeCameraModel()

    info_pubs = []
    for camera_name in camera_names:
        info_pubs.append([camera_name, rospy.Publisher(camera_name + '/camera_info', camera_info_messages.CameraInfo,
                                                       queue_size=10)])

    tb = TransformBroadcaster()

    move_by_delta = MoveByDelta(move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(move_group)
    move_by_absolute.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_camera_transforms(tb)
        publish_camera_info(info_pubs)
        publish_gripper_pixels(camera_model, move_group, gripper_publisher)
        rate.sleep()


if __name__ == "__main__":
    main()
