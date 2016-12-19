
#include "icp_laser/icp_laser.h"

using namespace CSM;

TransformWithFitness
icp_laser::find_by_csm()
{
	TransformWithFitness twf;

	twf.transform.setIdentity();
	twf.fitness = 1000;

	if(!we_have_laser || !we_have_map) return twf;

	sm_params input;
	sm_result result;

	LDP real_laser_ldp;
	LDP simulated_laser_ldp;

	geometry_msgs::Pose p = currentPose();

	sensor_msgs::LaserScan::Ptr simulated_scan = createSimulatedLaserScan(p);

	int v = scanToLDP(laser, real_laser_ldp, max_laser_point_width);


	if(v<30) 
	{
		ROS_INFO("not enough valid data");
		return twf;

	}

	v = scanToLDP(*simulated_scan, simulated_laser_ldp, max_simulated_point_width);

	if(v<30) 
	{
		ROS_INFO("not enough valid data");
		return twf;

	}

	#ifdef PUBLISH_SIMULATED_LASER_SCAN

	sensor_msgs::LaserScan windowed;
	ldpToScan(real_laser_ldp, windowed);
	laser_publisher.publish(windowed);

	ldpToScan(simulated_laser_ldp, windowed);
	sim_laser_publisher.publish(windowed);

	#endif


	ROS_INFO("aa");

	input.laser[0] = 0.0;
	input.laser[1] = 0.0;
	input.laser[2] = 0.0;

	input.laser_ref = simulated_laser_ldp;
	input.laser_sens = real_laser_ldp;
	input.first_guess[0] = 0;
	input.first_guess[1] = 0;
	input.first_guess[2] = 0;

	input.min_reading = laser.range_min;
	input.max_reading = laser.range_max;

	input.max_correspondence_dist = icp_max_correspondence_distance;

	input.epsilon_xy = icp_transformation_epsilon;
	input.epsilon_theta = icp_transformation_epsilon;

	input.max_iterations = icp_max_iterations;

	input.use_corr_tricks = 0;

	input.restart = 0;

	input.restart_threshold_mean_error = 0.01;

	input.restart_dt = 1.0;

	input.restart_dtheta = 0.1;

	input.clustering_threshold = 0.25;

	input.orientation_neighbourhood = 20;

	input.use_point_to_line_distance = 1;

	input.do_alpha_test = 0;

	input.outliers_maxPerc = 0.9;

	input.outliers_adaptive_order = 0.7;

	input.outliers_adaptive_mult = 2.0;

	input.use_sigma_weights = 0;

	input.use_ml_weights = 0;



	result.cov_x_m = 0;
	result.dx_dy1_m = 0;
	result.dx_dy2_m = 0;




	sm_icp(&input, &result);


	if (result.valid)
	{
		ROS_INFO("%f %f %f", result.x[0], result.x[1], result.x[2]);
		ROS_INFO("%f", result.error);

		int num_of_corr = 1;
		for(int i = 0; i<real_laser_ldp->nrays; i++)
		{
			if(real_laser_ldp->corr[i].valid) num_of_corr++;
		}
		twf.fitness = result.error/double(num_of_corr-1);
	}

	else 
	{
		ROS_INFO("invalid");
		twf.fitness = 1000;
	}

	tf::Transform correction;

	tf::Vector3 origin(result.x[0], result.x[1], 0);
	tf::Quaternion rotation;
	rotation.setRPY(0, 0, result.x[2]);

	correction.setOrigin(origin);
	correction.setRotation(rotation);

	twf.transform = base_transform*correction*laser_to_base;





	//Visualize correspondences
#ifdef PUBLISH_CORRESPONDENCES
	visualization_msgs::Marker correspondences_marker;
	correspondences_marker.header.frame_id = "/base_laser_link";
	correspondences_marker.header.stamp = ros::Time();
	correspondences_marker.ns = "icp_laser";
	correspondences_marker.id = 0;
	correspondences_marker.type = visualization_msgs::Marker::LINE_LIST;
//	correspondences_marker.action = visualization_msgs::Marker::ADD;

	correspondences_marker.color.g = 1;
	correspondences_marker.color.a = 1;

	correspondences_marker.scale.x = 0.01;


	for(int i = 0; i<real_laser_ldp->nrays; i++)
	{
		if(real_laser_ldp->corr[i].valid)
		{
			int ic = real_laser_ldp->corr[i].j1;

			if(simulated_laser_ldp->valid[ic])
			{
				geometry_msgs::Point pn;
				pn.x = real_laser_ldp->points[i].p[0];
				pn.y = real_laser_ldp->points[i].p[1];
				pn.z = 0;
				correspondences_marker.points.push_back(pn);

				

				//2ROS_INFO("%d -> %d", i, ic);
				

				pn.x = simulated_laser_ldp->points[ic].p[0];
				pn.y = simulated_laser_ldp->points[ic].p[1];
				correspondences_marker.points.push_back(pn);

			}



		}
	}

	cor_publisher.publish(correspondences_marker);

#endif




	ld_free(real_laser_ldp);
	ld_free(simulated_laser_ldp);

	gsl_matrix_free(result.cov_x_m);
	gsl_matrix_free(result.dx_dy1_m);
	gsl_matrix_free(result.dx_dy2_m);

	return twf;
}

int icp_laser::scanToLDP(sensor_msgs::LaserScan &scan, LDP &ldp, double width_threshold)
{




	int n = scan.ranges.size();
/*	for(int i = 0; i<scan.ranges.size(); i++)
	{
		double angle = scan.angle_increment*i + scan.angle_min;
		double local_y = sin(angle)*scan.ranges[i];
		double local_x = cos(angle)*scan.ranges[i];


		if(width_threshold != 0 && (fabs(local_y) > width_threshold/2 || local_x < 0 )) continue;

		else n++;

	}
*/
	int valids = 0;

	if(n<1) return 0;

	ldp = ld_alloc_new(n);


	for(int i = 0; i<n; i++)
	{
		double angle = scan.angle_increment*i + scan.angle_min;
		double local_y = sin(angle)*scan.ranges[i];
		double local_x = cos(angle)*scan.ranges[i];


		if(width_threshold != 0 && (fabs(local_y) > width_threshold/2 || local_x < 0 ))
		{
			ldp->valid[i] = 0;
			ldp->readings[i] = -1;
		}

		else 
		{

			ldp->valid[i] = 1;
			ldp->readings[i] = scan.ranges[i];

			valids++;

		}
		ldp->theta[i] = angle;
		ldp->cluster[i] = -1;

	}

	ldp->min_theta = ldp->theta[0];
	ldp->max_theta = ldp->theta[n-1];

	return valids;
}

void icp_laser::ldpToScan(LDP &ldp, sensor_msgs::LaserScan &scan)
{
	int n = ldp->nrays;

	scan = laser;

	


	for(int i = 0; i<scan.ranges.size(); i++)
	{
		scan.ranges[i] = 0;
	}

	for(int i = 0; i<n; i++)
	{
		double angle = ldp->theta[i];
		int index = round( (angle-scan.angle_min)/scan.angle_increment);
		scan.ranges[index] = ldp->readings[i];

	}
}