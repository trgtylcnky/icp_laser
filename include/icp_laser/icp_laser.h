
#ifndef ICP_LASER_H_DEFINED
#define ICP_LASER_H_DEFINED


#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"

#include "tf/transform_listener.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include "pcl_ros/point_cloud.h"

#include "occupancy_grid_utils/ray_tracer.h"

#include "laser_geometry/laser_geometry.h"

#include "dynamic_reconfigure/server.h"
#include "icp_laser/ICP_LaserConfig.h"

#include <csm/csm_all.h>


//Publish simulated laser scan for debug purposes
#define PUBLISH_SIMULATED_LASER_SCAN

#define PUBLISH_SIMULATED_LASER_CLOUD

#define PUBLISH_LASER_CLOUD

#define PUBLISH_CORRESPONDENCES

struct TransformWithFitness
{
	tf::Transform transform;
	double fitness;
};

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

	//Publishing data for visualization purposes
	#ifdef PUBLISH_SIMULATED_LASER_SCAN
	ros::Publisher sim_laser_publisher;
	ros::Publisher laser_publisher;
	ros::Publisher corrected_sim_publisher;
	#endif

	#ifdef PUBLISH_SIMULATED_LASER_CLOUD
	ros::Publisher sim_laser_cloud_publisher;
	#endif

	#ifdef PUBLISH_LASER_CLOUD
	ros::Publisher laser_cloud_publisher;
	ros::Publisher transformed_laser_cloud_publisher;
	#endif

	#ifdef PUBLISH_CORRESPONDENCES
	ros::Publisher cor_publisher;
	#endif

	//Neglect laser data far away from that distance
	double max_simulated_point_distance;
	double max_simulated_point_width;
	int min_simulated_point_count;
	double max_laser_point_distance;
	double max_laser_point_width;
	int min_laser_point_count;

	//ICP parameters
	double icp_max_correspondence_distance;
	unsigned int icp_max_iterations;
	double icp_transformation_epsilon;
	double icp_euclidean_distance_epsilon;

	double inlier_distance;

	double fitness_threshold;

	//Pose update parameters
	double max_jump_distance; //Do not publish the pose if it is too far away
	double min_jump_distance; //No need to publish new pose if it is too close
	double max_rotation; //Same for rotation
	double min_rotation;

	double update_interval;  //Limit the update frequency of pose
	ros::Time update_time;  //Holds the last time new pose published

	//covariance in published pose (PoseWithCovarianceStamped)
	double pose_covariance_xx;
	double pose_covariance_yy;
	double pose_covariance_aa;





	dynamic_reconfigure::Server<icp_laser_config::ICP_LaserConfig> *reconfigure_server;
	boost::recursive_mutex configuration_mutex_;

public:
	icp_laser();
	~icp_laser();

	//Create simulated laser data as if robot is at give pose
	sensor_msgs::LaserScan::Ptr createSimulatedLaserScan(geometry_msgs::Pose&);

	//Map callback
	void getMapFromTopic(const nav_msgs::OccupancyGrid::Ptr);
	//Laser scan callback
	void getLaserFromTopic(const sensor_msgs::LaserScan::Ptr);

	//get the current laser pose
	geometry_msgs::Pose currentPose();

	//performs the ICP algorithm and return the final transformation
	TransformWithFitness find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &);

	void dynamic_reconfigure_callback(icp_laser_config::ICP_LaserConfig &config, uint32_t level);


	void setICPParameters(double, unsigned int, double, double);
	void setLaserCloudLimits(double, double, int);
	void setSimulatedCloudLimits(double, double, int);
	void setJumpParameters(double, double, double, double);
	void setPoseCovariance(double, double, double);
	void setUpdateInterval(double);
	void setInlierThreshold(double);
	void setFitnessThreshold(double);

	//Update the robot pose according to the given transform
	void updatePose(TransformWithFitness);

	//convert laser scan to point cloud, laser scanner pose is given
	void laserToPCloud(
		sensor_msgs::LaserScan&, 
		geometry_msgs::Pose, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr&, 
		double,
		int);

	//convert transformation matrix to tf::Traansform object
	void matrixAsTransform (const Eigen::Matrix4f&,  tf::Transform&);

	TransformWithFitness find_by_csm();
	int scanToLDP(sensor_msgs::LaserScan &, LDP &, double);
	void ldpToScan(LDP &, sensor_msgs::LaserScan &);

};




#endif