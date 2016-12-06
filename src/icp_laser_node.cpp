#include "icp_laser/icp_laser.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icp_laser_node");
	ros::NodeHandle n;

	icp_laser il;
	il.setUpdateInterval(2);

	ros::Rate r(2);


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	
	while(ros::ok())
	{

		r.sleep();

		tf::Transform t = il.find(icp);
		il.updatePose(t);

		ros::spinOnce();

	}
	
	return 0;

}



