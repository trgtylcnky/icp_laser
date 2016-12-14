#include "icp_laser/icp_laser.h"

TransformWithFitness
icp_laser::find(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp)
{

	TransformWithFitness twf;

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
		laserToPCloud(*simulated, p, simulated_cloud, max_simulated_point_width, min_simulated_point_count);
		simulated_cloud->header.frame_id = "/map";



		#ifdef PUBLISH_SIMULATED_LASER_CLOUD
		sim_laser_cloud_publisher.publish(*simulated_cloud);
		#endif

		#ifdef PUBLISH_LASER_CLOUD
		laser_cloud->header.frame_id = "/base_link";
		laser_cloud_publisher.publish(*laser_cloud);
		#endif

		icp.setInputSource(laser_cloud);
		icp.setInputTarget(simulated_cloud);

		icp.setMaxCorrespondenceDistance (icp_max_correspondence_distance);
		icp.setMaximumIterations (icp_max_iterations);
		icp.setTransformationEpsilon (icp_transformation_epsilon);
		icp.setEuclideanFitnessEpsilon (icp_euclidean_distance_epsilon);

		pcl::PointCloud<pcl::PointXYZ> Final;

		tf::StampedTransform b_transform;

		tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
		tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), b_transform);


		tf::Matrix3x3 base_rot (b_transform.getRotation());
		tf::Vector3 base_transl = b_transform.getOrigin();

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
		f.header.frame_id = "/map";
		transformed_laser_cloud_publisher.publish(f);
		#endif

		std::vector<int> nn_indices (1);
		std::vector<float> nn_sqr_dists (1);

/*
		target_tree->setInputCloud(simulated_cloud);
		int num_of_inliers = 0;
		for (int i = 0; i<Final.points.size(); i++)
		{
			if(target_tree->radiusSearch(Final.points[i], inlier_distance, nn_indices, nn_sqr_dists, 1) != 0)
				num_of_inliers++;
		}

		twf.fitness = double(num_of_inliers) / double(laser_cloud->points.size());
*/
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

	twf.transform = t;
	twf.fitness = icp.getFitnessScore();
	
	return twf;

}