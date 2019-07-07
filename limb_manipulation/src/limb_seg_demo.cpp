#include <vector>

#include "limb_manipulation/region_growing_rgb_segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "limb_seg_demo");
  ros::NodeHandle nh;

  ros::Publisher colored_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  manipulation::Segmenter segmenter(colored_cloud_pub, marker_pub);
  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 1, &manipulation::Segmenter::JointStateCallback, &segmenter);
  ros::Subscriber segment_sub = 
      nh.subscribe("/head_camera/depth_registered/points", 1, &manipulation::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;
}
