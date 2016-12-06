#include "icp_laser/icp_laser.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icp_laser_node");
	ros::NodeHandle n;

	icp_laser il;


	ros::Rate r(10);


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



/*
		
		il.find(icp);

		if(icp.hasConverged())
		{
			p = il.poseFromICP(icp);
			posePublisher.publish(p);
		}
*/

/*
		tf::Transform correction;

		il.find(icp);



		if(icp.hasConverged()){

			try
			{
				tl.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
				tl.lookupTransform("/map", "/base_link", ros::Time(0), base_transform);
			}
			catch (tf::TransformException ex)
			{
			    ROS_ERROR("%s", ex.what());
			    continue;
			}

			ROS_INFO(" %f ", icp.getFitnessScore());
		
			il.matrixAsTransform (icp.getFinalTransformation(),  correction);

			tf::Transform t = correction * base_transform;

			ROS_INFO("%f %f %f",correction.getOrigin().x(), correction.getOrigin().y(), tf::getYaw(correction.getRotation()));


			geometry_msgs::PoseWithCovarianceStamped pose;

			pose.pose.pose.position.x = t.getOrigin().x();
			pose.pose.pose.position.y = t.getOrigin().y();

			pose.pose.pose.orientation.x = t.getRotation().getX();
			pose.pose.pose.orientation.y = t.getRotation().getY();
			pose.pose.pose.orientation.z = t.getRotation().getZ();
			pose.pose.pose.orientation.w = t.getRotation().getW();


			pose.pose.covariance[0] = 0.25;
			pose.pose.covariance[7] = 0.25;
			pose.pose.covariance[35] = 0.1;

			
			
			ROS_INFO("publishing pose");
			posePublisher.publish(pose);
			
			


		}


*/