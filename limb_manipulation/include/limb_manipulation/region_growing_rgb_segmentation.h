#include <iostream>
#include <vector>
#include <limits.h>
#include <math.h>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"

namespace manipulation {
class Segmenter {
 public:
  Segmenter(const ros::Publisher& colored_cloud_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher colored_cloud_pub_;
};
}  // namespace manipulation
