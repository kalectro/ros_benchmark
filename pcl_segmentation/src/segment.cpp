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

// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// declare timing variables
	ros::Time start, stop;	

	// measure time for converting the message
	start = ros::Time::now();
	pcl::fromROSMsg (*input, cloud);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;
	
	// measure time for making input cloud accessible
	start = ros::Time::now();
	seg.setInputCloud (cloud.makeShared ());
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;

	// measure time segment pointcloud
	start = ros::Time::now();
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

	// Set up SAC parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);	

	// Write description for csv file	
	cout << "time[ms];matching points" << endl;
	
	// Spin
	ros::spin ();
}
