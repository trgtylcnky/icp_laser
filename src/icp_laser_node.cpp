#include "icp_laser/icp_laser.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icp_laser_node");
	ros::NodeHandle n;

	icp_laser il;

	ros::Rate r(2);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	
	while(ros::ok())
	{

		r.sleep();
		ros::spinOnce();



		TransformWithFitness t = il.find_by_csm();
		il.updatePose(t);

		il.find_by_csm();


	}
	
	return 0;

}



