#include <iostream>
#include <string>

#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "pcl_conversions/pcl_conversions.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

void print_usage() {
  std::cout << "Saves a point cloud on head_kinect/depth_registered/points to "
               "NAME.bag in the DIRECTORY."
            << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME DIRECTORY" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "save_cloud_main");
  if (argc < 3) {
    print_usage();
    return 1;
  }
  std::string name(argv[1]);
  std::string directory(argv[2]);

  sensor_msgs::PointCloud2ConstPtr cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "head_camera/depth_registered/points");

  tf::TransformListener tf_listener;
  tf_listener.waitForTransform("base_link", cloud->header.frame_id,
                               ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform;
  try {
    tf_listener.lookupTransform("base_link", cloud->header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (tf::ExtrapolationException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  sensor_msgs::PointCloud2 cloud_out;
  pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);


  std::string filename(directory + name + ".bag");
  sensor_msgs::PointCloud2 final_cloud;

  // VERSION 1: concatenating previous cloud
  // // read the previous cloud
  // bool overwrite;
  // ros::param::get("overwrite_prev_bag", overwrite);
  // if (!overwrite) {
  //   rosbag::Bag prev_bag;
  //   sensor_msgs::PointCloud2::ConstPtr prev_cloud;
  //   try {
  //     prev_bag.open(filename);

  //     std::vector<std::string> topics;
  //     topics.push_back(std::string("head_camera/depth_registered/points"));
  //     rosbag::View view(prev_bag, rosbag::TopicQuery(topics));
  //     foreach (rosbag::MessageInstance const m, view) {
  //       prev_cloud = m.instantiate<sensor_msgs::PointCloud2>();
  //       // concatenate the previous cloud and the new cloud
  //       if (prev_cloud != NULL) {
  //         pcl::concatenatePointCloud(*prev_cloud, cloud_out, final_cloud);
  //       }
  //     }

  //     prev_bag.close();
  //   } catch (rosbag::BagException& e) {
  //     std::cerr << e.what() << std::endl;
  //   }
  // } else {
  //   ros::param::set("overwrite_prev_bag", false);
  // }

  // write the new cloud into bag file
  // rosbag::Bag bag;
  // bag.open(filename, rosbag::bagmode::Write);
  // bag.write("head_camera/depth_registered/points", ros::Time::now(), final_cloud);
  // bag.close();

  // VERSION 2: rewrite previous cloud everytime
  // write the new cloud into bag file
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  bag.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
  bag.close();

  return 0;
}
