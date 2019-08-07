#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX, PXAndTheta, Theta
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import camera_info_messages

# Created by Tim and modified by Xinyi

camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2, 'camera3': camera_info_messages.camera3}

# Xinyi modified: Change the camera angle for limb manipulation

# transform_broadcaster_mapping = {
#         'camera1': ((0.7, 0, 2.3), (-0.70711, -0.70711, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
#         'camera2': ((0.9, -1.2, 0.8), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
#         'camera3': ((1.7, -0.1, 0.8), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
#         }

transform_broadcaster_mapping = {
        'camera1': ((0.7, 0, 2.3), (-0.70711, -0.70711, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
        'camera2': ((0.9, -1.2, 0.8), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
        'camera3': ((1.7, -0.1, 0.8), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
        }
orientation_mapping = {'camera1': 2, 'camera2': 1, 'camera3': 1}
orientation_sign_mapping = {'camera1': -1, 'camera2': 1, 'camera3': 1}
camera_names = ['camera1', 'camera2', 'camera3']
pr2_move_group_name = "right_arm"


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def quat_array_to_quat(quat_array):
    new_quat = Quaternion()
    new_quat.x = quat_array[0]
    new_quat.y = quat_array[1]
    new_quat.z = quat_array[2]
    new_quat.w = quat_array[3]
    return new_quat


def publish_camera_transforms(tb):
    for key in transform_broadcaster_mapping:
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])


def publish_camera_transforms_debug(tb, pub):
    id = 0
    for key in transform_broadcaster_mapping:
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])
        # Xinyi modified for debug: visualize camera poses
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=id,
                pose=Pose(Point(transform_data[0][0], transform_data[0][1], transform_data[0][2]), 
                          Quaternion(transform_data[1][0], transform_data[1][1], transform_data[1][2], transform_data[1][3])),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=transform_data[3])
        pub.publish(marker)
        id += 1


def publish_camera_info(publishers):
    for pub in publishers:
        pub[1].publish(camera_info_mapping[pub[0]])


def publish_gripper_pixels(camera_model, move_group, pub):
    data_array = []

    ps = move_group.get_current_pose()

    for camera in camera_names:
        camera_model.fromCameraInfo(camera_info_mapping[camera])
        x, y, z = getCameraDistances(camera, ps)
        (u, v) = camera_model.project3dToPixel((x, y, z))
        data_array.append([camera, int(u), int(v)])

    for array in data_array:
        px_msg = PX()
        px_msg.camera_name = array[0]
        px_msg.pixel_x = array[1]
        px_msg.pixel_y = array[2]
        pub.publish(px_msg)


def getCameraDistances(camera_name, ps):
    if camera_name == "camera1":
        camera_location = transform_broadcaster_mapping["camera1"][0]
        z = camera_location[2] - ps.pose.position.z
        x = ps.pose.position.x - camera_location[0]
        y = camera_location[1] - ps.pose.position.y
    elif camera_name == "camera2":
        camera_location = transform_broadcaster_mapping["camera2"][0]
        z = camera_location[1] - ps.pose.position.y
        x = camera_location[0] - ps.pose.position.x
        y = camera_location[2] - ps.pose.position.z
    elif camera_name == "camera3":
        camera_location = transform_broadcaster_mapping["camera3"][0]
        z = camera_location[1] - ps.pose.position.y
        x = camera_location[0] - ps.pose.position.x
        y = camera_location[2] - ps.pose.position.z
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return x,y,z


