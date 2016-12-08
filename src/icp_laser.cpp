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

	laser_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
	laser_cloud->header.frame_id = "/map";
	
	max_simulated_point_distance = 3;
	min_simulated_point_count = 200;

	max_laser_point_distance = 3;
	min_laser_point_count = 200;

	icp_max_correspondence_distance = 0.5;
	icp_max_iterations = 2500;
	icp_transformation_epsilon = 1e-6;

	max_jump_distance = 0.8;
	min_jump_distance = 0.1;
	min_rotation = 0.025;
	max_rotation = 0.7;

	update_interval = 5;
	update_time = ros::Time::now();

	tf_listener.waitForTransform( "/base_laser_link", "base_link", ros::Time(0), ros::Duration(10.0));
	tf_listener.lookupTransform( "/base_laser_link", "base_link", ros::Time(0), laser_to_base);

	pose_covariance_xx = 0.02;
	pose_covariance_yy = 0.02;
	pose_covariance_aa = 0.005;

}



void icp_laser::getLaserFromTopic(const sensor_msgs::LaserScan::Ptr _laser)
{

	laser = *_laser;
	we_have_laser = true;

	//p is pose of laser respect to base
	geometry_msgs::Pose p;
	p.position.x = -laser_to_base.getOrigin().x();
	p.position.z = -laser_to_base.getOrigin().z();
	p.orientation.w =1;

	laserToPCloud(*_laser, p, laser_cloud, max_laser_point_distance, min_laser_point_count);

	try
	{
		tf_listener.waitForTransform("/map", "/base_laser_link", ros::Time(0), ros::Duration(10.0));
		tf_listener.lookupTransform("/map", "/base_laser_link", ros::Time(0), base_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}

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

	    tf::poseTFToMsg (laser_transform, pos);


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
		sim_laser_publisher.publish(*sml);
		#endif

	}
	return sml;

}

tf::Transform
icp_laser::find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp)
{

	

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

		//Simulated laser cloud is created on map
		//according to current map -> base_laser_link frame
		sensor_msgs::LaserScan::Ptr simulated = createSimulatedLaserScan(p);
		pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		laserToPCloud(*simulated, p, simulated_cloud, max_simulated_point_distance, min_simulated_point_count);
		simulated_cloud->header.frame_id = "/map";

		#ifdef PUBLISH_SIMULATED_LASER_CLOUD
		sim_laser_cloud_publisher.publish(*simulated_cloud);
		#endif

		#ifdef PUBLISH_LASER_CLOUD
		laser_cloud_publisher.publish(*laser_cloud);
		#endif

		icp.setInputSource(laser_cloud);
		icp.setInputTarget(simulated_cloud);

		icp.setMaxCorrespondenceDistance (icp_max_correspondence_distance);
		icp.setMaximumIterations (icp_max_iterations);
		icp.setTransformationEpsilon (icp_transformation_epsilon);
		icp.setEuclideanFitnessEpsilon (1e-6);

		pcl::PointCloud<pcl::PointXYZ> Final;

		tf::Matrix3x3 base_rot (base_transform.getRotation());
		tf::Vector3 base_transl = base_transform.getOrigin();

		//Converting current map -> base_laser_link transform to matrix
		//This matrix is initial guess for icp
		Eigen::Matrix4f tr_matrix = Eigen::Matrix4f::Identity();
		tr_matrix(0,0) = base_rot[0].x();
		tr_matrix(0,1) = base_rot[0].y();
		tr_matrix(0,2) = base_rot[0].z();
		tr_matrix(1,0) = base_rot[1].x();
		tr_matrix(1,1) = base_rot[1].y();
		tr_matrix(1,2) = base_rot[1].z();
		tr_matrix(2,0) = base_rot[2].x();
		tr_matrix(2,1) = base_rot[2].y();
		tr_matrix(2,2) = base_rot[2].z();

		tr_matrix(0,3) = base_transl.x();
		tr_matrix(1,3) = base_transl.y();
		tr_matrix(2,3) = base_transl.z();

		icp.align(Final, tr_matrix); //Perform icp with initial guess

		#ifdef PUBLISH_LASER_CLOUD
		pcl::PointCloud<pcl::PointXYZ> f;
		pcl::transformPointCloud (*laser_cloud, f, icp.getFinalTransformation());

		laser_cloud_publisher.publish(f);
		#endif

	}

	tf::Transform t;

	//if converge is found, return final transform
	//else, return the old pose
	if(icp.hasConverged())
	{
		matrixAsTransform (icp.getFinalTransformation(), t);
	}
	else 
	{
		t.setRotation(base_transform.getRotation());
		t.setOrigin(base_transform.getOrigin());
	}
	
	return t;

}

void icp_laser::updatePose(tf::Transform t)
{

	ros::Duration interval = ros::Time::now() - update_time;

	ROS_INFO("time: %f", interval.toSec());

	//calculate the difference between old and new transformations
	double diff_x = t.getOrigin().x() - (base_transform*laser_to_base).getOrigin().x();
	double diff_y = t.getOrigin().y() - (base_transform*laser_to_base).getOrigin().y();
	double diff_a = tf::getYaw(t.getRotation()) - tf::getYaw((base_transform*laser_to_base).getRotation());
	
	/*
	ROS_INFO("%f %f %f", 
		t.getOrigin().x(),
		t.getOrigin().y(),
		tf::getYaw(t.getRotation()));


*/
	ROS_INFO("%f %f %f", 
		diff_x,
		diff_y,
		diff_a);

	//conditions for publishing new pose
	if (interval.toSec() > update_interval &&
		abs(diff_x)<max_jump_distance &&
		abs(diff_y)<max_jump_distance &&
		abs(diff_a) < max_rotation &&

		(abs(diff_x)>min_jump_distance ||
		abs(diff_y)>min_jump_distance ||
		abs(diff_a) > min_rotation  )
		
		) 
	{
		
		update_time = ros::Time::now();


		geometry_msgs::PoseWithCovarianceStamped pose;


		tf::poseTFToMsg (t, pose.pose.pose);


		pose.pose.covariance[0] = pose_covariance_xx;
		pose.pose.covariance[7] = pose_covariance_yy;
		pose.pose.covariance[35] = pose_covariance_aa;
		

		pose.header.frame_id = "/map";
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
void icp_laser::setJumpParameters(double a, double b, double c, double d)
{
	max_jump_distance = a;
	min_jump_distance = b;
	max_rotation = c;
	min_rotation = d;
}
void icp_laser::setUpdateInterval(double t)
{
	update_interval = t;
}
void icp_laser::setPoseCovariance(double x, double y, double a)
{
	pose_covariance_aa = a;
	pose_covariance_yy = y;
	pose_covariance_xx = x;
}

void icp_laser::laserToPCloud(
	sensor_msgs::LaserScan& scan, 
	geometry_msgs::Pose pos, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& result, 
	double radius_threshold = 0,
	int count_threshold = 0)
{


	const double angle_range = scan.angle_max - scan.angle_min;
	const unsigned n = (unsigned) round(1+angle_range/scan.angle_increment);


	double laser_yaw = tf::getYaw(pos.orientation);



	//result->points.resize(n);


	result->points.clear();


	for (int i=0; i<n; i++)
	{
		if(radius_threshold != 0 && scan.ranges[i]>radius_threshold)
			continue;


		pcl::PointXYZ p;

		//Calclulate the point position
		//assuming 2D space
		double _yaw = laser_yaw + scan.angle_increment*i + scan.angle_min;
		double x = pos.position.x + cos(_yaw)*scan.ranges[i];
		double y = pos.position.y + sin(_yaw)*scan.ranges[i];
		p.x = x;
		p.y = y;
		p.z = pos.position.z;

		result->points.push_back(p);
	}

	//since no one prevents me from having fun with recursion
	//if there is no enough point, repeat 
	if(result->points.size() < count_threshold && radius_threshold<scan.range_max) 
		laserToPCloud(scan, pos, result, radius_threshold + 0.5, count_threshold);


}

void 
icp_laser::matrixAsTransform (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
{

	//I copied this from SnapMapICP

	
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

    bt = tf::Transform(basis,origin);
}

