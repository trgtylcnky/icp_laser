#include "icp_laser/icp_laser.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icp_laser_node");
	ros::NodeHandle n;

	
	

	icp_laser il;




	il.setUpdateInterval(1);

	il.setICPParameters(0.25, 2500, 1e-9, 1e-8);

	ros::Rate r(5);


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	
	while(ros::ok())
	{

		r.sleep();
		ros::spinOnce();

		TransformWithFitness t = il.find(icp);
		il.updatePose(t);

		

	}
	
	return 0;

}



