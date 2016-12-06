#include "icp_laser/icp_laser.h"
#include <cstdlib>


icp_laser::icp_laser()
{
	we_have_map = false;
	we_have_laser = false;
	
	map_subscriber = nodeHandle.subscribe("/map", 1000, &icp_laser::getMapFromTopic, this);

	laser_subscriber = nodeHandle.subscribe("/scan", 1000, &icp_laser::getLaserFromTopic, this);

	pose_publisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);

	#ifdef PUBLISH_SIMULATED_LASER_SCAN
	sim_laser_publisher = nodeHandle.advertise<sensor_msgs::LaserScan>("/icp_laser/simulated_scan", 1000);
	#endif

	#ifdef PUBLISH_SIMULATED_LASER_CLOUD
	sim_laser_cloud_publisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/icp_laser/simulated_cloud", 1000);
	#endif

	#ifdef PUBLISH_LASER_CLOUD
	laser_cloud_publisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/icp_laser/laser_cloud", 1000);
	#endif

	max_simulated_point_distance = 2;
	min_simulated_point_count = 200;

	max_laser_point_distance = 2;
	min_laser_point_count = 200;

	icp_max_correspondence_distance = 0.2;
	icp_max_iterations = 50;
	icp_transformation_epsilon = 1e-7;

	max_jump_distance = 0.8;
	min_jump_distance = 0.1;

	update_interval = 5;
	update_time = ros::Time::now();

}



void icp_laser::getLaserFromTopic(const sensor_msgs::LaserScan::Ptr _laser)
{

	laser = *_laser;
	we_have_laser = true;
}

void icp_laser::getMapFromTopic(const nav_msgs::OccupancyGrid::Ptr _map)
{

	map = *_map;
	we_have_map = true;
}

geometry_msgs::Pose icp_laser::currentPose()
{
	tf::StampedTransform laser_transform;
	geometry_msgs::Pose pos;

	try
	{
	    tf_listener.waitForTransform("/map", "/base_laser_link", ros::Time(0), ros::Duration(10.0));
	    tf_listener.lookupTransform("/map", "/base_laser_link", ros::Time(0), laser_transform);

	    pos.position.x = laser_transform.getOrigin().x();
	    pos.position.y = laser_transform.getOrigin().y();
	    pos.position.z = laser_transform.getOrigin().z();
	    pos.orientation.x = laser_transform.getRotation().getX();
	    pos.orientation.y = laser_transform.getRotation().getY();
	    pos.orientation.z = laser_transform.getRotation().getZ();
	    pos.orientation.w = laser_transform.getRotation().getW();


	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s", ex.what());
	}

	return pos;
}

sensor_msgs::LaserScan::Ptr icp_laser::createSimulatedLaserScan(geometry_msgs::Pose& pos)
{

	sensor_msgs::LaserScan::Ptr sml;

	if(we_have_map && we_have_laser)
	{

		sml = occupancy_grid_utils::simulateRangeScan(map, pos, laser, false);

		#ifdef PUBLISH_SIMULATED_LASER_SCAN
		sim_laser_publisher.publish(*scan);
		#endif

	}
	return sml;

}

tf::Transform
icp_laser::find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp)
{
	ROS_INFO("in: find");
	

	if(!we_have_laser)
	{
		ROS_ERROR("Trying to find icp without laser data");
	}
	if(!we_have_map)
	{
		ROS_ERROR("Trying to find icp without map data");
	}
	if(we_have_map && we_have_laser)
	{
		geometry_msgs::Pose p = currentPose();

		sensor_msgs::LaserScan::Ptr simulated = createSimulatedLaserScan(p);

		pcl::PointCloud<pcl::PointXYZ>::Ptr real_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		
		laserToPCloud(laser, p, real_cloud, max_laser_point_distance, min_laser_point_count);
		laserToPCloud(*simulated, p, simulated_cloud, max_simulated_point_distance, min_simulated_point_count);

		//reducePoints(real_cloud, 0.02);
		//reducePoints(simulated_cloud, 0.02);

		real_cloud->header.frame_id = "/map";
		simulated_cloud->header.frame_id = "/map";

		#ifdef PUBLISH_SIMULATED_LASER_CLOUD
		sim_laser_cloud_publisher.publish(*simulated_cloud);
		#endif

		#ifdef PUBLISH_LASER_CLOUD
		laser_cloud_publisher.publish(*real_cloud);
		#endif

		icp.setInputSource(real_cloud);
		icp.setInputTarget(simulated_cloud);

		icp.setMaxCorrespondenceDistance (icp_max_correspondence_distance);
		icp.setMaximumIterations (icp_max_iterations);
		icp.setTransformationEpsilon (icp_transformation_epsilon);

		pcl::PointCloud<pcl::PointXYZ> Final;

		icp.align(Final);

	}

	tf::Transform t;

	if(icp.hasConverged())
	{
		matrixAsTransform (icp.getFinalTransformation(), t);
	}

	
	return t;

}

void icp_laser::updatePose(tf::Transform t)
{

	ros::Duration interval = ros::Time::now() - update_time;

	ROS_INFO("time: %f", interval.toSec());
	ROS_INFO("%f %f %f", 
		abs(t.getOrigin().x()),
		t.getOrigin().x(),
		tf::getYaw(t.getRotation()));


	if (interval.toSec() > update_interval &&
		abs(t.getOrigin().x())<max_jump_distance &&
		abs(t.getOrigin().y())<max_jump_distance &&

		(abs(t.getOrigin().x())>min_jump_distance ||
		abs(t.getOrigin().y())>min_jump_distance 
		) 
		) 
	{

		update_time = ros::Time::now();

		tf::StampedTransform base_transform;

		try
		{
			tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
			tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), base_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}

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
		ROS_INFO("publishing..");

		pose_publisher.publish(pose);

	}


}


void icp_laser::setICPParameters(double a, unsigned int b, double c)
{
	icp_max_correspondence_distance = a;
	icp_max_iterations = b;
	icp_transformation_epsilon = c;
}
void icp_laser::setLaserCloudParameters(double a, int b, double c, int d)
{
	max_simulated_point_distance = a;
	min_simulated_point_count = b;
	max_simulated_point_distance = c;
	min_simulated_point_count = d;
}
