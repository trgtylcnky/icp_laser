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
	min_jump_distance = 0.02;
	min_rotation = 0.02;
	max_rotation = 0.7;

	last_fitness = 1;
//	current_fitness = 1;

	update_interval = 5;
	update_time = ros::Time::now();

	tf_listener.waitForTransform( "/base_laser_link", "base_link", ros::Time(0), ros::Duration(10.0));
	tf_listener.lookupTransform( "/base_laser_link", "base_link", ros::Time(0), laser_to_base);

	pose_covariance_xx = 0.025;
	pose_covariance_yy = 0.025;
	pose_covariance_aa = 0.005;

}



void icp_laser::getLaserFromTopic(const sensor_msgs::LaserScan::Ptr _laser)
{

	laser = *_laser;
	we_have_laser = true;

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

		//pcl::PointCloud<pcl::PointXYZ>::Ptr real_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		
		//laserToPCloud(laser, p, real_cloud, max_laser_point_distance, min_laser_point_count);
		laserToPCloud(*simulated, p, simulated_cloud, max_simulated_point_distance, min_simulated_point_count);

		//reducePoints(real_cloud, 0.02);
		//reducePoints(simulated_cloud, 0.02);

		//real_cloud->header.frame_id = "/map";
		simulated_cloud->header.frame_id = "/map";

		#ifdef PUBLISH_SIMULATED_LASER_CLOUD
		sim_laser_cloud_publisher.publish(*simulated_cloud);
		#endif

		#ifdef PUBLISH_LASER_CLOUD
		laser_cloud_publisher.publish(*laser_cloud);
		//laser_cloud_publisher.publish(*real_cloud);
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

		icp.align(Final, tr_matrix);

		#ifdef PUBLISH_LASER_CLOUD
		pcl::PointCloud<pcl::PointXYZ> f;
		pcl::transformPointCloud (*laser_cloud, f, icp.getFinalTransformation());

		laser_cloud_publisher.publish(f);
		#endif

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
		t.getOrigin().x(),
		t.getOrigin().y(),
		tf::getYaw(t.getRotation()));

	tf::Transform diff = t.inverseTimes(base_transform);


	if (interval.toSec() > update_interval &&
		abs(diff.getOrigin().x())<max_jump_distance &&
		abs(diff.getOrigin().y())<max_jump_distance &&
		abs(tf::getYaw(diff.getRotation())) < max_rotation &&

		(abs(diff.getOrigin().x())>min_jump_distance ||
		abs(diff.getOrigin().y())>min_jump_distance ||
		abs(tf::getYaw(diff.getRotation())) > min_rotation
		) 
		) 
	{
		/*
		trans_que[trans_que_index] = t;
		trans_que_index++;
		if(trans_que_index == trans_que.size()) trans_que_index = 0;

		double x, y, yaw;
		x=y=yaw=0;

		for(int i=0; i<trans_que.size(); i++)
		{
			x+=trans_que[i].getOrigin().x();
			y+=trans_que[i].getOrigin().y();
			yaw+=tf::getYaw(trans_que[i].getRotation());
		}

		x=x/double(trans_que.size());
		y=y/double(trans_que.size());
		yaw=yaw/double(trans_que.size());

		t.setOrigin(tf::Vector3(x, y, 0));
		t.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));
*/
		update_time = ros::Time::now();

		

		//t =  base_transform * t;
		//t = laser_to_base * t ;

		geometry_msgs::PoseWithCovarianceStamped pose;


		tf::poseTFToMsg (t, pose.pose.pose);

		double yaw = tf::getYaw(t.getRotation()) ;
		//pose.pose.pose.position.x +=  cos(yaw)*laser_to_base.getOrigin().x();
		//pose.pose.pose.position.y +=  sin(yaw)*laser_to_base.getOrigin().x();
/*
		double yaw = tf::getYaw(t.getRotation()) ;

		tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, yaw), pose.pose.pose.orientation);


		pose.pose.pose.orientation.x = t.getRotation().getX();
		pose.pose.pose.orientation.y = t.getRotation().getY();
		pose.pose.pose.orientation.z = t.getRotation().getZ();
		pose.pose.pose.orientation.w = t.getRotation().getW();
*/

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

	ROS_INFO("in: laserToPCloud");
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
icp_laser::matrixAsTransform (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
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

    bt = tf::Transform(basis,origin);
}

geometry_msgs::PoseWithCovarianceStamped 
icp_laser::poseFromICP(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp)
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

void icp_laser::reducePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float dist)
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

