#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

ros::Publisher pub;

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "create_pointcloud");
	ros::NodeHandle nh;

	ros::Rate loop_rate(1);
	
	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	while (ros::ok())
	{
		// load the file
		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("benchmark.pcd", *cloud) == -1) 
		{
			PCL_ERROR ("Couldn't read file benchmark.pcd \n");
			return (-1);
		}

		// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg (*cloud, output);

		// Publish the model coefficients
		pub.publish (output);

		// Spin
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
