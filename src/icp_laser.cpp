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
	laser_publisher = nodeHandle.advertise<sensor_msgs::LaserScan>("/icp_laser/scan", 1000);
	#endif

	#ifdef PUBLISH_SIMULATED_LASER_CLOUD
	sim_laser_cloud_publisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/icp_laser/simulated_cloud", 1000);
	#endif

	#ifdef PUBLISH_LASER_CLOUD
	laser_cloud_publisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/icp_laser/laser_cloud", 1000);
	transformed_laser_cloud_publisher= nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/icp_laser/transformed_laser_cloud", 1000);
	#endif

	#ifdef PUBLISH_CORRESPONDENCES
	cor_publisher = nodeHandle.advertise<visualization_msgs::Marker>("/icp_laser/correspondences", 10);
	#endif

	laser_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
	laser_cloud->header.frame_id = "/map";


	
	max_simulated_point_distance = 8;
	max_simulated_point_width = 2;
	min_simulated_point_count = 200;

	max_laser_point_distance = 8;
	max_laser_point_width = 1;
	min_laser_point_count = 200;

	icp_max_correspondence_distance = 0.5;
	icp_max_iterations = 25;
	icp_transformation_epsilon = 1e-8;
	icp_euclidean_distance_epsilon = 1e-5;

	inlier_distance = 0.1;

	max_jump_distance = 0.8;
	min_jump_distance = 0.03;
	min_rotation = 0.025;
	max_rotation = 0.7;

	update_interval = 5;
	update_time = ros::Time::now();
	try
	{
		tf_listener.waitForTransform( "/base_laser_link", "base_link", ros::Time(0), ros::Duration(10.0));
		tf_listener.lookupTransform( "/base_laser_link", "base_link", ros::Time(0), laser_to_base);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	pose_covariance_xx = 0.01;
	pose_covariance_yy = 0.01;
	pose_covariance_aa = 0.005;

	fitness_threshold = 0.001;

	dynamic_reconfigure::Server<icp_laser_config::ICP_LaserConfig>::CallbackType cb;

	reconfigure_server = new dynamic_reconfigure::Server<icp_laser_config::ICP_LaserConfig>(ros::NodeHandle("~"));
	cb = boost::bind(&icp_laser::dynamic_reconfigure_callback, this, _1, _2);
	reconfigure_server->setCallback(cb);

}

icp_laser::~icp_laser()
{
	delete reconfigure_server;

}

void icp_laser::dynamic_reconfigure_callback(icp_laser_config::ICP_LaserConfig &config, uint32_t level)
{
	boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

	icp_max_iterations = config.icp_max_iterations;
	icp_max_correspondence_distance = config.icp_max_correspondence_distance;
	icp_transformation_epsilon = config.icp_transformation_epsilon;
	icp_euclidean_distance_epsilon = config.icp_euclidean_distance_epsilon;

	max_laser_point_width = config.max_laser_point_width;
	max_simulated_point_width = config.max_simulated_point_width;

	pose_covariance_aa = config.pose_covariance_aa;
	pose_covariance_yy = config.pose_covariance_loc;
	pose_covariance_xx = config.pose_covariance_loc;

	update_interval = config.update_interval;

	fitness_threshold = config.fitness_threshold;

	inlier_distance = config.inlier_distance;

	
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

	laserToPCloud(*_laser, p, laser_cloud, max_laser_point_width, min_laser_point_count);

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

	sensor_msgs::LaserScan scanner_info = laser;
	//scanner_info.angle_min = - 3.14;
//	scanner_info.angle_max = 3.14;

	if(we_have_map && we_have_laser)
	{

		sml = occupancy_grid_utils::simulateRangeScan(map, pos, scanner_info, false);

	//	#ifdef PUBLISH_SIMULATED_LASER_SCAN
	//	sim_laser_publisher.publish(*sml);
	//	#endif

	}

	return sml;

}



void icp_laser::updatePose(TransformWithFitness twf)
{
	tf::Transform t = twf.transform;

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
	ROS_INFO("Fitness Score: %f ", twf.fitness);

	if(twf.fitness > fitness_threshold) ROS_INFO("Not fit Enough!!");

	//conditions for publishing new pose
	else if (
		interval.toSec() > update_interval &&
		fabs(diff_x)<max_jump_distance &&
		fabs(diff_y)<max_jump_distance &&
		fabs(diff_a) < max_rotation &&

		(fabs(diff_x)>min_jump_distance ||
		fabs(diff_y)>min_jump_distance ||
		fabs(diff_a) > min_rotation  )
		
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


void icp_laser::setICPParameters(double a, unsigned int b, double c, double d)
{
	icp_max_correspondence_distance = a;
	icp_max_iterations = b;
	icp_transformation_epsilon = c;
	icp_euclidean_distance_epsilon = d;
}
void icp_laser::setLaserCloudLimits(double a, double b, int c)
{
	max_laser_point_distance = a;
	max_laser_point_width = b;
	min_laser_point_count = c;
}
void icp_laser::setSimulatedCloudLimits(double a, double b, int c)
{
	max_simulated_point_distance = a;
	max_simulated_point_width = b;
	min_simulated_point_count = c;
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
void icp_laser::setInlierThreshold(double i)
{
	inlier_distance = i;
}
void icp_laser::setFitnessThreshold(double f)
{
	fitness_threshold = f;
}

void icp_laser::laserToPCloud(
	sensor_msgs::LaserScan& scan, 
	geometry_msgs::Pose pos, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& result, 
	double width_threshold = 0,
	int count_threshold = 0)
{


	const double angle_range = scan.angle_max - scan.angle_min;
	const unsigned n = (unsigned) round(1+angle_range/scan.angle_increment);


	double laser_yaw = tf::getYaw(pos.orientation);



	//result->points.resize(n);


	result->points.clear();


	for (int i=0; i<n; i++)
	{
		


		pcl::PointXYZ p;

		//Calclulate the point position
		//assuming 2D space
		double angle = scan.angle_increment*i + scan.angle_min;
		double local_y = sin(angle)*scan.ranges[i];
		double local_x = cos(angle)*scan.ranges[i];


		if(width_threshold != 0 && (fabs(local_y) > width_threshold/2 || local_x < 0 )) continue;

		double x = pos.position.x + cos(laser_yaw + angle)*scan.ranges[i];
		double y = pos.position.y + sin(laser_yaw + angle)*scan.ranges[i];


		p.x = x;
		p.y = y;
		p.z = pos.position.z;

		result->points.push_back(p);
	}

/*
	//since no one prevents me from having fun with recursion
	//if there is no enough point, repeat 
	if(result->points.size() < count_threshold && radius_threshold<scan.range_max) 
		laserToPCloud(scan, pos, result, width_threshold + 0.5, count_threshold);

*/
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

