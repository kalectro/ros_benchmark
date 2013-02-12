#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

// Initialize ROS
ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ros::Time start, stop;	
	start = ros::Time::now();
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud.makeShared ());
	seg.segment (*inliers, *coefficients);

	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;
	
	// output number of points found on a surface
	cout << inliers->indices.size() << endl;
	
	// Publish the model coefficients
	pub.publish (coefficients);
}

int
main (int argc, char** argv)
{
	ros::init (argc, argv, "segment");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("pointcloud", 1, cloud_cb);
	
	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<pcl::ModelCoefficients> ("coefficients", 1);

	// Write description for csv file	
	cout << "time[ms];matching points" << endl;
	
	// Spin
	ros::spin ();
}
