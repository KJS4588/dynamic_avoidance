#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"
#include "std_msgs/Bool.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"

#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "math.h"
#include "vector"
#define _USE_MATH_DEFINES
#define DIST 3.5

using namespace std;
typedef pcl::PointXYZI PointType;

class DynamicAvoidance {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, point_pub_, acker_pub_, stop_pub_, finish_pub_;
    ros::Subscriber sub_, odom_sub_, point_sub_;

	vector<geometry_msgs::Point> obs_;
	std_msgs::Bool stop_flag_, finish_flag_;
	
	bool dynamic_finish_ = false;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);
    vector<geometry_msgs::Point> clustering(const sensor_msgs::PointCloud2ConstPtr &input, double min_x, double max_x, double min_y, double max_y);
	double calcDist(geometry_msgs::Point p){ return sqrt(p.x*p.x + p.y*p.y + p.z*p.z); }

};

