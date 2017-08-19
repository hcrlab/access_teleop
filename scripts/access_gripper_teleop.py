#! /usr/bin/env python

import rospy
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX
import fetch_api
from image_geometry import PinholeCameraModel
import camera_info_messages

big_z_mappings = {'camera1': 2.5, 'camera2': 1.5}
camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2}

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def publish_camera_transforms():
    pass


def publish_camera_info():
    for key in camera_info_mapping:
        pub = rospy.Publisher(key + '/camera_info', camera_info_messages.CameraInfo, queue_size=10)
        pub.publish(camera_info_mapping[key])





class MoveByDelta(object):
    def __init__(self, arm, gripper):
        self._arm = arm
        self._gripper = gripper

    def start(self):
        rospy.Subscriber('/access_teleop', DeltaPX, self.callback)

    def callback(self, data):
        dx = data.delta_x
        dy = data.delta_y
        camera_name = data.camera_name
        print("The dx is " + str(dx) + " the dy is " + str(dy) + " and the camera name is " + camera_name)
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_mapping[camera_name])
        x, y, z = camera_model.projectPixelTo3dRay((dx, dy))
        x_center, y_center, z_center = camera_model.projectPixelTo3dRay((0, 0))
        big_z = big_z_mappings[camera_name]
        big_x = (x / z) * big_z  #Use law of similar trianges to solve
        big_y = (y / z) * big_z

        big_x_center = (x_center / z_center) * big_z
        big_y_center = (y_center / z_center) * big_z

        print("The x_center  is " + str(x_center) + " the y_center  is " + str(y_center) + " and the z_center is " + str(z_center))
        print("The x distance is " + str(big_x) + " the y distance is " + str(big_y) + " and the camera name is " + camera_name)
        print("The x distance from big_x to big_x_center is " + str(big_x - big_x_center) + " the y distance is " + str(big_y - big_y_center))

        x_distance = big_x - big_x_center
        y_distance = big_y - big_y_center




class MoveByAbsolute(object):
    def __init__(self, arm, gripper):
        self._arm = arm
        self._gripper = gripper

    def start(self):
        pass


def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    move_by_delta = MoveByDelta(arm, gripper)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(arm, gripper)
    move_by_absolute.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_camera_transforms()
        publish_camera_info()
        rate.sleep()


if __name__ == "__main__":
    main()
