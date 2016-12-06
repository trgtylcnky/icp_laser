

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
//#define PUBLISH_SIMULATED_LASER_SCAN

//#define PUBLISH_SIMULATED_LASER_CLOUD

//#define PUBLISH_LASER_CLOUD

class icp_laser
{

	//Stores the map taken from /map topic
	nav_msgs::OccupancyGrid map;

	//It is true if we get map from /map topic
	bool we_have_map;

	//Stores the laser scanner data taken from /scan topic
	sensor_msgs::LaserScan laser;

	//It is true if we get laser data
	bool we_have_laser;

	tf::TransformListener tf_listener;
	
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

	double update_interval;
	ros::Time update_time;

public:
	icp_laser();
	void publishSimulatedLaserScan(sensor_msgs::LaserScan::Ptr);
	sensor_msgs::LaserScan::Ptr createSimulatedLaserScan(geometry_msgs::Pose&);
	void getMapFromTopic(const nav_msgs::OccupancyGrid::Ptr);
	void getLaserFromTopic(const sensor_msgs::LaserScan::Ptr);
	geometry_msgs::Pose currentPose();
	tf::Transform find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &);

	void setICPParameters(double, unsigned int, double);
	void setLaserCloudParameters(double, int, double, int);

	void updatePose(tf::Transform);


	void laserToPCloud(
		sensor_msgs::LaserScan& scan, 
		geometry_msgs::Pose pos, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr result, 
		double radius_threshold = 0,
		int count_threshold = 0)
	{

		ROS_INFO("in: laserToPCloud");
		const double angle_range = scan.angle_max - scan.angle_min;
		const unsigned n = (unsigned) round(1+angle_range/scan.angle_increment);


		double laser_yaw = tf::getYaw(pos.orientation);

		ROS_INFO("0");

		ROS_INFO("%lu", result->points.size());

		//result->points.resize(n);


		ROS_INFO("%u %lu", n, scan.ranges.size());

		result->points.clear();


		for (int i=0; i<n; i++)
		{
			if(radius_threshold != 0 && scan.ranges[i]>radius_threshold)
				continue;

			pcl::PointXYZ p;
			double _yaw = laser_yaw + scan.angle_increment*i + scan.angle_min;
			double x = pos.position.x + cos(_yaw)*scan.ranges[i];
			double y = pos.position.y + sin(_yaw)*scan.ranges[i];
			p.x = x;
			p.y = y;
			p.z = pos.position.z;

			result->points.push_back(p);
		}

		//since no one prevents me from having fun with recursion
		if(result->points.size() < count_threshold) 
			laserToPCloud(scan, pos, result, radius_threshold + 0.5, count_threshold);


		ROS_INFO("out: laserToPCloud");


	}

	void 
	matrixAsTransform (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
	{
	    double mv[12];

	    mv[0] = out_mat (0, 0) ;
	    mv[4] = out_mat (0, 1);
	    mv[8] = out_mat (0, 2);
	    mv[1] = out_mat (1, 0) ;
	    mv[5] = out_mat (1, 1);
	    mv[9] = out_mat (1, 2);
	    mv[2] = out_mat (2, 0) ;
	    mv[6] = out_mat (2, 1);
	    mv[10] = out_mat (2, 2);

	    tf::Matrix3x3 basis;
	    basis.setFromOpenGLSubMatrix(mv);
	    tf::Vector3 origin(out_mat (0, 3),out_mat (1, 3),out_mat (2, 3));

	    ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

	    bt = tf::Transform(basis,origin);
	}

	geometry_msgs::PoseWithCovarianceStamped 
	poseFromICP(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp)
	{

		tf::TransformListener tl;
		tf::Transform t;
		tf::StampedTransform base_transform;
		try
		{
			tl.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
			tl.lookupTransform("/map", "/base_link", ros::Time(0), base_transform);
		}
		catch (tf::TransformException ex)
		{
		    ROS_ERROR("%s", ex.what());
		}

		matrixAsTransform (icp.getFinalTransformation(),  t);

		t = t * base_transform;

		geometry_msgs::PoseWithCovarianceStamped pose;

		pose.pose.pose.position.x = t.getOrigin().x();
		pose.pose.pose.position.y = t.getOrigin().y();

		pose.pose.pose.orientation.x = t.getRotation().getX();
		pose.pose.pose.orientation.y = t.getRotation().getY();
		pose.pose.pose.orientation.z = t.getRotation().getZ();
		pose.pose.pose.orientation.w = t.getRotation().getW();


		pose.pose.covariance[0] = 0.4;
		pose.pose.covariance[7] = 0.4;
		pose.pose.covariance[35] = 0.1;

		return pose;

	}

	void reducePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float dist)
	{
		std::vector<pcl::PointXYZ> p;

		p.push_back(cloud->points[0]);

		for(int i=1; i < cloud->points.size(); i++)
		{
			if(cloud->points[i].x > p[i-1].x + dist
				|| cloud->points[i].x < p[i-1].x - dist
				|| cloud->points[i].y > p[i-1].y + dist
				|| cloud->points[i].y < p[i-1].y - dist)
			{
				p.push_back(cloud->points[i]);
			}
		}

		cloud->points.resize(p.size());
		for(int i=0; i<p.size(); i++)
		{
			cloud->points[i] = p[i];
		}
	}



};


double abs(double d)
{
	if(d < 0) return -d;
	else return d;
}