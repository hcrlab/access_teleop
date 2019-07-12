#include <vector>
#include <iostream>
#include <sstream>

#include "limb_manipulation/region_growing_rgb_segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception/downsample.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "limb_seg_demo");
  ros::NodeHandle nh;

  // cloud used in color segmentation
  ros::Publisher filtered_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  // color-segmented cloud
  ros::Publisher colored_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1, true); 
  // marker
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  manipulation::Segmenter segmenter(filtered_cloud_pub, colored_cloud_pub, marker_pub);

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 1, &manipulation::Segmenter::JointStateCallback, &segmenter);
  
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1, true);
  perception::Downsampler downsampler(downsample_pub);
  
  ros::Subscriber downsample_sub =
  //     nh.subscribe("/mock_point_cloud", 1, &perception::Downsampler::Callback, &downsampler);  // For sim purpose
      nh.subscribe("/head_camera/depth_registered/points", 1, &perception::Downsampler::Callback, &downsampler);  // For real purpose
  
  ros::Subscriber segment_sub = 
      nh.subscribe("/downsampled_cloud", 1, &manipulation::Segmenter::Callback, &segmenter);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
