#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

ros::Publisher pub;
string pcd_path;

// Parameters used if not available on parameter server
static const string PCD_PATH = "src/ros_benchmark/pcl_segmentation/pcds/benchmark.pcd";

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "create_pointcloud");
	ros::NodeHandle nh;

	ros::Rate loop_rate(0.1);
	
	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	while (ros::ok())
	{
		// get pcd path
		if (nh.getParam("create_pointcloud/pcd_path", pcd_path))
		{
			ROS_DEBUG("Found pcd path %s on server", pcd_path.c_str());
		}
		else
		{
			pcd_path = PCD_PATH;
			ROS_DEBUG("Found no pcd path on server, trying %s", pcd_path.c_str());
		}

		// load the pcd
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path, *cloud) == -1) 
		{
			PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
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
