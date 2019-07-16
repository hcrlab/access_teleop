#include "limb_manipulation/region_growing_rgb_segmentation.h"
#include <ros/console.h>

#include "perception/box_fitter.h"
#include "perception/object.h"
#include "visualization_msgs/Marker.h"
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/filters/extract_indices.h>
#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

#include <boost/lexical_cast.hpp>

namespace manipulation {

Segmenter::Segmenter(const ros::Publisher& filtered_cloud_pub, const ros::Publisher& colored_cloud_pub, const ros::Publisher& marker_pub)
  : filtered_cloud_pub_(filtered_cloud_pub), colored_cloud_pub_(colored_cloud_pub), marker_pub_(marker_pub) {}


void Segmenter::JointStateCallback(const sensor_msgs::JointState& msg) {
  head_tilt_joint_value_ = msg.position[5];
  torso_lift_joint_value_ = msg.position[2];
}


void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  SegmentPointCloud(msg);
}


void Segmenter::SegmentPointCloud(const sensor_msgs::PointCloud2& msg) {
  // parameters
  int reg_distance, reg_point_color, reg_region_color, min_cluster_size, max_cluster_size;
  ros::param::param("reg_distance", reg_distance, 10);
  ros::param::param("reg_point_color", reg_point_color, 6);
  ros::param::param("reg_region_color", reg_region_color, 5);
  ros::param::param("min_cluster_size", min_cluster_size, 60);
  ros::param::param("max_cluster_size", max_cluster_size, 10000);

  // pass through filter
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_unfiltered (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (msg, *cloud_unfiltered);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, *indices);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pass.filter (*filtered_cloud);


  // publish the filtered point cloud
  // sensor_msgs::PointCloud2 filtered_cloud_msg;
  // pcl::toROSMsg (*filtered_cloud, filtered_cloud_msg);
  // filtered_cloud_pub_.publish(filtered_cloud_msg);


  // color segmentation
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (filtered_cloud);
  // reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (reg_distance);  // determine whether the point is in the neighboring or not
  reg.setPointColorThreshold (reg_point_color);
  reg.setRegionColorThreshold (reg_region_color);
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize (max_cluster_size);
  
  std::vector <pcl::PointIndices> object_indices;
  reg.extract (object_indices);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  

  // convert to sensor message
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (*colored_cloud, cloud_out);

  // **********************************
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
  // **********************************

  // publish the color-segmented cloud
  colored_cloud_pub_.publish(colored_cloud_transformed);

  // convert ROS message to pcl PointCloud type for labelling
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_with_transformation (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (colored_cloud_transformed, *colored_cloud_with_transformation);
  // label the markers
  SegmentMarker(&object_indices, colored_cloud_with_transformation);

  
}

void Segmenter::SegmentMarker(std::vector <pcl::PointIndices>* object_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  // parameters
  int size_min, size_max;
  ros::param::param("size_min", size_min, 3);
  ros::param::param("size_max", size_max, 10);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);

  int num = 0;  // for the id# of visualization markers

  for (size_t i = 0; i < object_indices->size(); ++i) {
    // reify indices into a point cloud of the object
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices->at(i);

    if (indices->indices.size() > size_min && indices->indices.size() < size_max) {
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
      object_marker.id = num;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::SPHERE;

      object_marker.pose.position.x = centroid[0];
      object_marker.pose.position.y = centroid[1];
      object_marker.pose.position.z = centroid[2];
      object_marker.scale.x = 0.05;
      object_marker.scale.y = 0.05;
      object_marker.scale.z = 0.05;
      object_marker.color.g = 1;
      object_marker.color.a = 1;
      marker_pub_.publish(object_marker);

      num++;

    }

    /*
    // Fit every region into a bounding box, and only label markers with specific sizes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // fill in object_cloud using indices
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*object_cloud);

    // bounding box
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose pose;
    simple_grasping::extractShape(*object_cloud, *object_cloud_out, shape, pose);

    if (shape.type == shape_msgs::SolidPrimitive::BOX) {
      double x = shape.dimensions[0], y = shape.dimensions[1], z = shape.dimensions[2];

      if (z > 0.5) {
        std::cout << num << std::endl;
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "z: " << z << std::endl;
        std::cout << std::endl;

        // sensor_msgs::PointCloud2 object_cloud_transformed;
        // pcl::toROSMsg(*object_cloud, object_cloud_transformed);
        // colored_cloud_pub_.publish(object_cloud_transformed);

        // find the center of the object
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*object_cloud, centroid);

        // publish a marker
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = num;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;


        object_marker.text = boost::lexical_cast<std::string>(num);
        object_marker.pose.position.x = centroid[0];
        object_marker.pose.position.y = centroid[1];
        object_marker.pose.position.z = centroid[2];
        object_marker.scale.x = 0.05;
        object_marker.scale.y = 0.05;
        object_marker.scale.z = 0.05;
        object_marker.color.g = 1;
        object_marker.color.a = 1;
        marker_pub_.publish(object_marker);

        num++;
      }
    }
    */
  }
}
}  // namespace manipulation
