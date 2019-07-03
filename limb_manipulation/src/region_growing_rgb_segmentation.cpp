#include "limb_manipulation/region_growing_rgb_segmentation.h"

#include <ros/console.h>

namespace manipulation {

Segmenter::Segmenter(const ros::Publisher& colored_cloud_pub)
    : colored_cloud_pub_(colored_cloud_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  // segmentation
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (msg, *cloud);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 100000.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);  // determine whether the point is in the neighboring or not
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  
  // convert to sensor message
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (*colored_cloud, cloud_out);

  // publish
  cloud_out.header.frame_id = "base_link";
  colored_cloud_pub_.publish(cloud_out);

  // // transform point cloud
  // tf::TransformListener tf_listener;
  // tf::StampedTransform transform;
  // try {
  //   tf_listener.waitForTransform("base_link", cloud_out.header.frame_id,
  //                                cloud_out.header.stamp, ros::Duration(5.0));
  //   tf_listener.lookupTransform("base_link", cloud_out.header.frame_id,
  //                               cloud_out.header.stamp, transform);
  // } catch (tf::LookupException& e) {
  //   ROS_ERROR("%s", e.what());
  // } catch (tf::ExtrapolationException& e) {
  //   ROS_ERROR("%s", e.what());
  // }

  // sensor_msgs::PointCloud2 colored_cloud_transformed;
  // pcl_ros::transformPointCloud("base_link", cloud_out, colored_cloud_transformed, tf_listener);


  // colored_cloud_pub_.publish(colored_cloud_transformed);

}
}  // namespace manipulation
