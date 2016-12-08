

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "tf/transform_listener.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include "pcl_ros/point_cloud.h"

#include "occupancy_grid_utils/ray_tracer.h"


//Publish simulated laser scan for debug purposes
#define PUBLISH_SIMULATED_LASER_SCAN

#define PUBLISH_SIMULATED_LASER_CLOUD

#define PUBLISH_LASER_CLOUD



class icp_laser
{

	//Stores the map taken from /map topic
	nav_msgs::OccupancyGrid map;

	//It is true if we get map from /map topic
	bool we_have_map;

	//Stores the laser scanner data taken from /scan topic
	sensor_msgs::LaserScan laser;
	pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud;

	//It is true if we get laser data
	bool we_have_laser;

	tf::TransformListener tf_listener;
	tf::StampedTransform laser_to_base;
	tf::StampedTransform base_transform;
	
	ros::NodeHandle nodeHandle;
	ros::Subscriber map_subscriber;
	ros::Subscriber laser_subscriber;
	ros::Publisher pose_publisher;

	#ifdef PUBLISH_SIMULATED_LASER_SCAN
	ros::Publisher sim_laser_publisher;
	#endif

	#ifdef PUBLISH_SIMULATED_LASER_CLOUD
	ros::Publisher sim_laser_cloud_publisher;
	#endif

	#ifdef PUBLISH_LASER_CLOUD
	ros::Publisher laser_cloud_publisher;
	#endif


	double max_simulated_point_distance;
	int min_simulated_point_count;

	double max_laser_point_distance;
	int min_laser_point_count;


	double icp_max_correspondence_distance;
	unsigned int icp_max_iterations;
	double icp_transformation_epsilon;

	double max_jump_distance;
	double min_jump_distance;
	double max_rotation;
	double min_rotation;

	double update_interval;
	ros::Time update_time;

	double last_fitness;

	double pose_covariance_xx;
	double pose_covariance_yy;
	double pose_covariance_aa;




public:
	icp_laser();

	sensor_msgs::LaserScan::Ptr createSimulatedLaserScan(geometry_msgs::Pose&);
	void getMapFromTopic(const nav_msgs::OccupancyGrid::Ptr);
	void getLaserFromTopic(const sensor_msgs::LaserScan::Ptr);
	geometry_msgs::Pose currentPose();
	tf::Transform find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &);

	void setICPParameters(double, unsigned int, double);
	void setLaserCloudParameters(double, int, double, int);

	void setJumpParameters(double, double, double, double);

	void setPoseCovariance(double, double, double);

	void setUpdateInterval(double);

	void updatePose(tf::Transform);


	void laserToPCloud(
		sensor_msgs::LaserScan&, 
		geometry_msgs::Pose, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr&, 
		double,
		int);


	void 
	matrixAsTransform (const Eigen::Matrix4f&,  tf::Transform&);


	geometry_msgs::PoseWithCovarianceStamped 
	poseFromICP(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>&);


	void reducePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float dist);

};


double abs(double d)
{
	if(d < 0) return -d;
	else return d;
}