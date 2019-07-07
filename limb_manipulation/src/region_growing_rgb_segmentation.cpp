#include "limb_manipulation/region_growing_rgb_segmentation.h"
#include <ros/console.h>

#include "perception/box_fitter.h"
#include "perception/object.h"
#include "visualization_msgs/Marker.h"
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/filters/extract_indices.h>

namespace manipulation {

Segmenter::Segmenter(const ros::Publisher& colored_cloud_pub, const ros::Publisher& marker_pub)
  : colored_cloud_pub_(colored_cloud_pub), marker_pub_(marker_pub) {}


void Segmenter::JointStateCallback(const sensor_msgs::JointState& msg) {
  head_tilt_joint_value_ = msg.position[5];
  torso_lift_joint_value_ = msg.position[2];
}


void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  SegmentPointCloud(msg);

}


void Segmenter::SegmentPointCloud(const sensor_msgs::PointCloud2& msg) {
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
  
  std::vector <pcl::PointIndices> object_indices;
  reg.extract (object_indices);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  
  // ********************************
  // convert to sensor message
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (*colored_cloud, cloud_out);

  // // transform point cloud
  // cloud_out.header.frame_id = "head_camera_rgb_frame";

  // tf::TransformListener tf_listener;
  // tf::StampedTransform transform;
  // try {
  //   tf_listener.waitForTransform("base_link", cloud_out.header.frame_id,
  //                                ros::Time(0), ros::Duration(5.0));
  //   tf_listener.lookupTransform("base_link", cloud_out.header.frame_id,
  //                               ros::Time(0), transform);
  // } catch (tf::LookupException& e) {
  //   ROS_ERROR("%s", e.what());
  // } catch (tf::ExtrapolationException& e) {
  //   ROS_ERROR("%s", e.what());
  // }

  // sensor_msgs::PointCloud2 colored_cloud_transformed;
  // pcl_ros::transformPointCloud("base_link", cloud_out, colored_cloud_transformed, tf_listener);

  // colored_cloud_pub_.publish(colored_cloud_transformed);
  // **********************************

  // transform point cloud
  tf::Transform rotation_transform;
  rotation_transform.setOrigin( tf::Vector3(0.17, 0.02, 1.03 + torso_lift_joint_value_) );
  tf::Quaternion q;
  q.setRPY(-1.5707 - head_tilt_joint_value_, 0, -1.5707);
  rotation_transform.setRotation(q);

  // // Eigen::Affine3f rotation_transform = Eigen::Affine3f::Identity();
  // // Define a translation of 2.5 meters on the x axis.
  // rotation_transform.translation() << 2.5, 0.0, 0.0;
  // // The same rotation matrix as before; theta radians around Z axis
  // rotation_transform.rotate (Eigen::AngleAxisf (1.5707, Eigen::Vector3f::UnitZ()));

  sensor_msgs::PointCloud2 colored_cloud_transformed;
  pcl_ros::transformPointCloud("base_link", rotation_transform, cloud_out, colored_cloud_transformed);

  // publish
  colored_cloud_pub_.publish(colored_cloud_transformed);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_with_transformation (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (colored_cloud_transformed, *colored_cloud_with_transformation);
  SegmentMarker(&object_indices, colored_cloud_with_transformation);
}

void Segmenter::SegmentMarker(std::vector <pcl::PointIndices>* object_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);

  for (size_t i = 0; i < object_indices->size(); ++i) {
    // reify indices into a point cloud of the object
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices->at(i);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // fill in object_cloud using indices
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*object_cloud);
    
    // find the center of the object
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*object_cloud, centroid);

    // publish a marker
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    object_marker.text = "0";
    object_marker.pose.position.x = centroid[0];
    object_marker.pose.position.y = centroid[1];
    object_marker.pose.position.z = centroid[2];
    object_marker.scale.x = 0.05;
    object_marker.scale.y = 0.05;
    object_marker.scale.z = 0.05;
    object_marker.color.g = 1;
    object_marker.color.a = 1;
    marker_pub_.publish(object_marker);
  }
}
}  // namespace manipulation
