#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from pprint import pprint
import perception


frozen = False
save_next = False
frozen_cloud = None
cloud_subscriber = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def republish(data, pub):
    global save_next
    global frozen
    if save_next:
        global frozen_cloud
        frozen_cloud = data
        cloud_subscriber.unregister()
        frozen = True
    rospy.loginfo("Publishing dynamic cloud")
    pub.publish(data)


def save_and_pub(data, pub):
    global save_next
    global frozen
    global cloud_subscriber
    if data.data:
        save_next = True
    else:
        frozen = False
        save_next = False
        cloud_subscriber = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, callback=republish,
                                            callback_args=pub, queue_size=1)


def main():
    global cloud_subscriber

    rospy.init_node('access_pointcloud_pub')
    wait_for_time()
    pub = rospy.Publisher('access_teleop/point_cloud', PointCloud2, queue_size=1)
    cloud_subscriber = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, callback=republish,
                                        callback_args=pub, queue_size=1)
    freeze_subscriber = rospy.Subscriber('/access_teleop/freeze_cloud', Bool, callback=save_and_pub, callback_args=pub,
                                         queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if frozen:
            rospy.loginfo("Publishing frozen cloud")
            pub.publish(frozen_cloud)
        rate.sleep()

if __name__ == '__main__':
    main()
