#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CompressedImage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

// Publish_topic
extern const string pubMergedTopic = "/test_publish/merged";
extern const string pubTransformedTopic = "/test_publish/transformed";
extern const string frameID = "world";


// Front_center
extern const string pointCloudTopic_fc = "/front64/pandar";
extern const string frameID_fc = "front64";

// Front_top
extern const string pointCloudTopic_ft = "/front32/pandar";
extern const string frameID_ft = "front32";

// Front_right
// extern const string pointCloudTopic_fr = "/front_r/pandar";
extern const string pointCloudTopic_fr = "/pandar161/pandar";
extern const string frameID_fr = "world";

// Front_left
// extern const string pointCloudTopic_fl = "/front_l/pandar";
extern const string pointCloudTopic_fl = "/pandar163/pandar";
extern const string frameID_fl = "world";

// Rear_right
extern const string pointCloudTopic_rr = "/rear_r/pandar";
extern const string frameID_rr = "world";

// Rear_left
extern const string pointCloudTopic_rl = "/rear_l/pandar";
extern const string frameID_rl = "world";

