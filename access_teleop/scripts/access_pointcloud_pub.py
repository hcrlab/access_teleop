#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from pprint import pprint
import perception
import rosbag


frozen = False
save_next = False
frozen_cloud = None
cloud_subscriber = None
world_pointcloud = []

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
    # rospy.loginfo("Publishing dynamic cloud")
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


def bag_file_reader():
    global world_pointcloud

    # read point cloud
    bag = rosbag.Bag('/home/maru/catkin_ws/src/access_teleop/bags/world.bag')  # $(rospack find access_teleop)/bags/world.bag')
    for topic, msg, t in bag.read_messages(topics=['head_camera/depth_registered/points']):
        world_pointcloud.append(msg)
    bag.close()

    rospy.set_param("bag_file_refreshed", "false")


def main():
    global cloud_subscriber
    global frozen

    rospy.init_node('pointcloud_saver')
    wait_for_time()
    pub = rospy.Publisher('access_teleop/point_cloud', PointCloud2, queue_size=1)

    world_pub = rospy.Publisher('/access_teleop/world_cloud', PointCloud2, queue_size=5)
    rospy.sleep(0.5)

    cloud_subscriber = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, callback=republish,
                                        callback_args=pub, queue_size=1)
    freeze_subscriber = rospy.Subscriber('/access_teleop/freeze_cloud', Bool, callback=save_and_pub, callback_args=pub,
                                         queue_size=1)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if frozen:
            # rospy.loginfo("Publishing frozen cloud")
            pub.publish(frozen_cloud)
        
        # check if bag file is refreshed
        if rospy.get_param("bag_file_refreshed"):
            bag_file_reader()
            # publish point cloud
            for cloud in world_pointcloud:
                world_pub.publish(cloud)

        rate.sleep()

if __name__ == '__main__':
    main()
