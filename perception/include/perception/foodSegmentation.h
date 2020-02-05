#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <vector>

#include "pcl/filters/extract_indices.h"
#include "pcl/ModelCoefficients.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "perception/object.h"

#include "perception/object_recognizer.h"

namespace perception_food {

static float OBJ_X_MIN = 0.15;
static float OBJ_X_MAX = 0.25;
static float OBJ_Y_MIN = 0.15;
static float OBJ_Y_MAX = 0.30;
static float OBJ_Z_MIN = 0.05;
static float OBJ_Z_MAX = 0.15;

// Does a complete tabletop segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the surface and the objects above it.
//  objects: The output objects.
// NOTE FOR OBJECT RECOGNITION: recognizer cannot be "const", otherwise it would discards qualifiers of Recognizer() 
void SegmentFoodScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    std::vector<perception::Object>* objects,
                    const ros::Publisher& marker_pub_p,
                    const ros::Publisher& surface_points_pub,
                    const ros::Publisher& above_surface_pub,
                    const ros::Publisher& food_pub,
                    perception::ObjectRecognizer& recognizer);

bool checkRange(const float x, const float y, const float z);

class FoodDetector {
 public:
  FoodDetector(const ros::Publisher& surface_points_pub,
                const ros::Publisher& marker_pub,
                const ros::Publisher& above_surface_pub,
                const ros::Publisher& food_pub,
                const perception::ObjectRecognizer& recognizer);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher above_surface_pub_;
  ros::Publisher food_pub_;
  perception::ObjectRecognizer recognizer_;
};
}  // namespace perception_food