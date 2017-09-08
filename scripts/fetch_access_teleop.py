#! /usr/bin/env python

import rospy
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX, PXAndTheta
import fetch_api
import camera_info_messages
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2}
transform_broadcaster_mapping = {'camera1': ((0.5, 0, 3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
                                'camera2': ((0.3, -1.5, 0.5), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link')}
orientation_mapping = {'camera1': 2, 'camera2': 1}
orientation_sign_mapping = {'camera1': -1, 'camera2': 1}

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

def publish_camera_transforms():
    for key in transform_broadcaster_mapping:
        tb = TransformBroadcaster()
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])


def publish_camera_info():
    for key in camera_info_mapping:
        pub = rospy.Publisher(key + '/camera_info', camera_info_messages.CameraInfo, queue_size=10)
        pub.publish(camera_info_mapping[key])


def dpx_to_distance(dx, dy, camera_name, current_ps, offset):
    print("The dx is " + str(dx) + " the dy is " + str(dy) + " and the camera name is " + camera_name)

    big_z_mappings = {'camera1': transform_broadcaster_mapping['camera1'][0][2] - current_ps.pose.position.z,
                      'camera2': transform_broadcaster_mapping['camera2'][0][1] - current_ps.pose.position.y}

    print("The frame_id for the current pose is " + current_ps.header.frame_id)
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info_mapping[camera_name])
    x, y, z = camera_model.projectPixelTo3dRay((dx, dy))
    print " x : {} , y : {} , z : {}".format(x, y, z)
    x_center, y_center, z_center = camera_model.projectPixelTo3dRay((0, 0))
    big_z = abs(big_z_mappings[camera_name])
    print("The big_z for " + camera_name + " is " + str(big_z))
    big_x = (x / z) * big_z  # Use law of similar trianges to solve
    big_y = (y / z) * big_z

    big_x_center = (x_center / z_center) * big_z
    big_y_center = (y_center / z_center) * big_z

    print("The x_center  is " + str(x_center) + " the y_center  is " + str(y_center) + " and the z_center is " + str(
        z_center))
    print(
    "The x distance is " + str(big_x - big_x_center) + " the y distance is " + str(big_y - big_y_center) + " and the camera name is " + camera_name + "\n")
    if offset:
        return big_x - big_x_center, big_y - big_y_center
    else:
        return big_x, big_y


def delta_modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        original_pose_stamped.pose.position.x += x_distance  # These directions came from looking at the cameras in rviz
        original_pose_stamped.pose.position.y -= y_distance
    elif camera_name == 'camera2':
        original_pose_stamped.pose.position.x += x_distance
        original_pose_stamped.pose.position.z -= y_distance
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
    else:
        raise ValueError('Did not pass in a valid camera_name')

    int_marker.pose.orientation.w = 1
    controls.markers.append(box_marker)
    int_marker.controls.append(controls)
    self._im_server.insert(int_marker)
    self._im_server.applyChanges()
    print("Marker changes should now be applied")


class MoveByDelta(object):
    def __init__(self, arm, gripper, move_group):
        self._arm = arm
        self._gripper = gripper
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback, queue_size=1)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = delta_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class MoveByAbsolute(object):
    def __init__(self, arm, gripper, move_group):
        self._arm = arm
        self._gripper = gripper
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=2)

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback, queue_size=1)

    def absolute_callback(self, data):
        print("We got the pixel with x of " + str(data.pixel_x) + " and y of " + str(data.pixel_y))
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        #add_marker(x_distance, y_distance, ps, data.camera_name, self)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)

class MoveAndOrient(object):
    def __init__(self, arm, gripper, move_group):
        self._arm = arm
        self._gripper = gripper
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

def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    move_group = MoveGroupCommander("arm")

    move_by_delta = MoveByDelta(arm, gripper, move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(arm, gripper, move_group)
    move_by_absolute.start()

    move_and_orient = MoveAndOrient(arm, gripper, move_group)
    move_and_orient.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_camera_transforms()
        publish_camera_info()
        rate.sleep()


if __name__ == "__main__":
    main()
