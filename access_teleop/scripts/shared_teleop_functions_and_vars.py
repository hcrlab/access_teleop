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


camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2, 'camera3': camera_info_messages.camera3}

# Xinyi added: Change the camera angle here
# original:
# transform_broadcaster_mapping = {'camera1': ((0.5, -0.3, 2.6), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
#                                 'camera2': ((0.7, -2.0, 0.55), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link')}
# transform_broadcaster_mapping = {
#         'camera1': ((0.7, 0, 2.3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
#         'camera2': ((0.9, -1.2, 1.1), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
#         'camera3': ((1.7, -0.1, 1.1), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
#         }

# with shelf:
transform_broadcaster_mapping = {
        'camera1': ((0.7, 0, 2.3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
        'camera2': ((0.9, -1.2, 1.1), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
        'camera3': ((1.7, -0.1, 1.1), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
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


def publish_camera_transforms(tb, pub):
    id = 0
    for key in transform_broadcaster_mapping:
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])
        # # Debug: visualize camera poses
        # marker = Marker(
        #         type=Marker.TEXT_VIEW_FACING,
        #         id=id,
        #         pose=Pose(Point(transform_data[0][0], transform_data[0][1], transform_data[0][2]), 
        #                   Quaternion(transform_data[1][0], transform_data[1][1], transform_data[1][2], transform_data[1][3])),
        #         scale=Vector3(0.06, 0.06, 0.06),
        #         header=Header(frame_id='base_link'),
        #         color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
        #         text=transform_data[3])
        # pub.publish(marker)
        # id += 1
        

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


def dpx_to_distance(dx, dy, camera_name, current_ps, offset):
    print("The dx is " + str(dx) + " the dy is " + str(dy) + " and the camera name is " + camera_name)

    big_z_mappings = {'camera1': transform_broadcaster_mapping['camera1'][0][2] - current_ps.pose.position.z, # x-y plane
                      'camera2': transform_broadcaster_mapping['camera2'][0][1] - current_ps.pose.position.y, # x-z plane
                      'camera3': transform_broadcaster_mapping['camera3'][0][0] - current_ps.pose.position.x} # y-z plane

    #print("The frame_id for the current pose is " + current_ps.header.frame_id)
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info_mapping[camera_name])
    x, y, z = camera_model.projectPixelTo3dRay((dx, dy))
    #print " x : {} , y : {} , z : {}".format(x, y, z)
    x_center, y_center, z_center = camera_model.projectPixelTo3dRay((0, 0))
    big_z = abs(big_z_mappings[camera_name])
    #print("The big_z for " + camera_name + " is " + str(big_z))
    big_x = (x / z) * big_z  # Use law of similar trianges to solve
    big_y = (y / z) * big_z

    big_x_center = (x_center / z_center) * big_z
    big_y_center = (y_center / z_center) * big_z

    #print("The x_center  is " + str(x_center) + " the y_center  is " + str(y_center) + " and the z_center is " + str(
    #    z_center))
    #print(
    #"The x distance is " + str(big_x - big_x_center) + " the y distance is " + str(big_y - big_y_center) + " and the camera name is " + camera_name + "\n")
    if offset:
        return big_x - big_x_center, big_y - big_y_center
    else:
        return big_x, big_y


def delta_modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        modified_ps.pose.position.x += x_distance  # These directions came from looking at the cameras in rviz
        modified_ps.pose.position.y -= y_distance
    elif camera_name == 'camera2':
        modified_ps.pose.position.x += x_distance
        modified_ps.pose.position.z -= y_distance
    elif camera_name == 'camera3':
        modified_ps.pose.position.y += x_distance
        modified_ps.pose.position.z -= y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return modified_ps


def absolute_modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        modified_ps.pose.position.x = transform_broadcaster_mapping['camera1'][0][0] + x_distance
        modified_ps.pose.position.y = transform_broadcaster_mapping['camera1'][0][1] - y_distance
    elif camera_name == 'camera2':
        modified_ps.pose.position.x = transform_broadcaster_mapping['camera2'][0][0] + x_distance
        modified_ps.pose.position.z = transform_broadcaster_mapping['camera2'][0][2] - y_distance
    elif camera_name == 'camera3':
        modified_ps.pose.position.y = transform_broadcaster_mapping['camera3'][0][1] + x_distance
        modified_ps.pose.position.z = transform_broadcaster_mapping['camera3'][0][2] - y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return modified_ps

def add_marker(x_distance, y_distance, ps, camera_name, self):
    controls = InteractiveMarkerControl()

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    int_marker = InteractiveMarker()
    int_marker.header = ps.header
    int_marker.name = "click_location"

    if camera_name == 'camera1':
        int_marker.pose.position.z = ps.pose.position.z
        int_marker.pose.position.x = transform_broadcaster_mapping['camera1'][0][0] + x_distance
        int_marker.pose.position.y = transform_broadcaster_mapping['camera1'][0][1] - y_distance
    elif camera_name == 'camera2':
        int_marker.pose.position.y = ps.pose.position.y
        int_marker.pose.position.x = transform_broadcaster_mapping['camera2'][0][0] + x_distance
        int_marker.pose.position.z = transform_broadcaster_mapping['camera2'][0][2] - y_distance
    elif camera_name == 'camera3':
        int_marker.pose.position.x = ps.pose.position.x
        int_marker.pose.position.y = transform_broadcaster_mapping['camera3'][0][1] + x_distance
        int_marker.pose.position.z = transform_broadcaster_mapping['camera3'][0][2] - y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')

    int_marker.pose.orientation.w = 1
    controls.markers.append(box_marker)
    int_marker.controls.append(controls)
    self._im_server.insert(int_marker)
    self._im_server.applyChanges()
    print("Marker changes should now be applied")


def addSetback(setback, theta, camera_name, ps):
    modified_ps = ps
    delta_y = math.sin(theta) * setback
    delta_x = math.cos(theta) * setback
    if camera_name == 'camera1':   # x-y plane
        modified_ps.pose.position.x -= delta_x
        modified_ps.pose.position.y += delta_y
    elif camera_name == 'camera2': # x-z plane
        modified_ps.pose.position.x -= delta_x
        modified_ps.pose.position.z += delta_y
    elif camera_name == 'camera3': # y-z plane
        modified_ps.pose.position.y -= delta_x
        modified_ps.pose.position.z += delta_y
    else:
        raise ValueError('Did not pass in a valid camera_name')


    return modified_ps